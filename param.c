#include "param.h"
#include "helpers.h"

#include <libopencm3/stm32/flash.h>

#define PARAM_FORMAT_VERSION 0

struct param_header_data_t {
    uint16_t format_version;
    uint16_t seq;
};

struct param_header_t {
    struct param_header_data_t data;
    uint16_t data_crc16;
};

struct param_tuple_t {
    enum param_key_t key;
    float value;
};

struct param_journal_entry_t {
    struct param_tuple_t data;
    uint16_t data_crc16;
    uint16_t invalid;
};

#define ENTRIES_PER_PAGE ((2048-sizeof(struct param_header_t))/sizeof(struct param_journal_entry_t))

struct param_page_t {
    struct param_header_t header;
    struct param_journal_entry_t journal[ENTRIES_PER_PAGE];
};

static struct param_page_t page_A __attribute__((section(".param_page_a")));
static struct param_page_t page_B __attribute__((section(".param_page_b")));
static struct param_page_t* page_in_use = NULL;
static bool squash_needed = false;

// initialize to default param value
static float param_cache[NUM_PARAMS] = {
    1.0f
};

static uint8_t param_has_entry[NUM_PARAMS];

static struct param_page_t* param_get_most_recent_valid_page(void);
static bool param_append_to_page(const struct param_page_t* page, enum param_key_t key, float value);
static bool param_flash_program_half_word(uint16_t* addr, uint16_t value);
static bool param_flash_erase_page(uint32_t addr);
static uint16_t param_get_header_data_crc16(struct param_header_data_t* header_data);
static uint16_t param_get_tuple_crc16(struct param_tuple_t* tuple);
static bool param_key_valid(enum param_key_t key);

void param_erase(void) {
    flash_unlock();
    param_flash_erase_page((uint32_t)&page_A);
    param_flash_erase_page((uint32_t)&page_B);
    flash_lock();
}

void param_init(void)
{
    uint8_t i;
    page_in_use = param_get_most_recent_valid_page();

    if (page_in_use == NULL) {
        flash_unlock();
        param_flash_erase_page((uint32_t)&page_A);
        param_flash_program_half_word(&page_A.header.data.format_version, PARAM_FORMAT_VERSION);
        param_flash_program_half_word(&page_A.header.data.seq, 0);
        param_flash_program_half_word(&page_A.header.data_crc16, param_get_header_data_crc16(&page_A.header.data));
        flash_lock();
        page_in_use = &page_A;
        return;
    }

    // load parameter values and determine if the journal needs to be squashed
    for(i=0; i<ENTRIES_PER_PAGE; i++) {
        if (page_in_use->journal[i].invalid) {
            break;
        }
        if (param_key_valid(page_in_use->journal[i].data.key) && page_in_use->journal[i].data_crc16 == param_get_tuple_crc16(&(page_in_use->journal[i].data))) {
            param_cache[page_in_use->journal[i].data.key] = page_in_use->journal[i].data.value;
            if (param_has_entry[page_in_use->journal[i].data.key]) {
                squash_needed = true;
            }
            param_has_entry[page_in_use->journal[i].data.key] = true;
        } else {
            squash_needed = true;
        }
    }

    // squash the journal if required
    param_squash();
}

float param_retrieve(enum param_key_t key)
{
    return param_cache[key];
}

bool param_set_and_save(enum param_key_t key, float value)
{
    if (page_in_use == NULL || !param_key_valid(key)) {
        return false;
    }

    bool success;
    success = param_append_to_page(page_in_use, key, value);
    if (!success && squash_needed) {
        param_squash();
        success = param_append_to_page(page_in_use, key, value);
    }

    if(success) {
        param_cache[key] = value;
    }

    return success;
}

static bool param_key_valid(enum param_key_t key)
{
    switch(key) {
        case PARAM_ID_TEST:
            return true;
        case NUM_PARAMS:
            break; // prevents warning
        }
    return false;
}

static struct param_page_t* param_get_most_recent_valid_page(void) {
    bool page_A_valid = page_A.header.data.format_version == PARAM_FORMAT_VERSION && page_A.header.data.seq != 0xFFFFU && page_A.header.data_crc16 == param_get_header_data_crc16(&page_A.header.data);
    bool page_B_valid = page_B.header.data.format_version == PARAM_FORMAT_VERSION && page_B.header.data.seq != 0xFFFFU && page_B.header.data_crc16 == param_get_header_data_crc16(&page_B.header.data);

    if (!page_A_valid && !page_B_valid) {
        return NULL;
    }

    if (!page_B_valid || (int16_t)(page_A.header.data.seq-page_B.header.data.seq) > 0) {
        return &page_A;
    } else {
        return &page_B;
    }
}

void param_squash(void)
{
    if (!squash_needed) {
        return;
    }

    uint8_t i=0;

    struct param_page_t* next_page = (page_in_use == &page_A) ? &page_B : &page_A;

    flash_unlock();
    param_flash_erase_page((uint32_t)next_page);
    flash_lock();

    for(i=0; i<NUM_PARAMS; i++) {
        if(param_key_valid(i) && param_has_entry[i]) {
            param_append_to_page(next_page, i, param_cache[i]);
        }
    }

    flash_unlock();
    param_flash_program_half_word(&next_page->header.data.format_version, PARAM_FORMAT_VERSION);
    param_flash_program_half_word(&next_page->header.data.seq, (page_in_use->header.data.seq+1)%(0xFFFFU-1));
    param_flash_program_half_word(&next_page->header.data_crc16, param_get_header_data_crc16(&next_page->header.data));
    flash_lock();

    page_in_use = next_page;
}

static bool param_append_to_page(const struct param_page_t* page, enum param_key_t key, float value)
{
    uint8_t i=0;

    struct param_journal_entry_t entry;

    entry.data.key = key;
    entry.data.value = value;
    entry.data_crc16 = param_get_tuple_crc16(&(entry.data));
    entry.invalid = 0;

    bool already_set = false;

    while (true) {
        if (i >= ENTRIES_PER_PAGE) {
            return false;
        }
        if (page->journal[i].invalid) {
            break;
        }
        already_set = page->journal[i].data.value == entry.data.value;
        i++;
    }
    if (already_set) {
        return true;
    }

    uint16_t* src_buf = (uint16_t*)&entry;
    uint16_t* dest_buf = (uint16_t*)&page->journal[i];
    uint8_t n = sizeof(entry)/2;

    flash_unlock();
    for(i=0; i<n; i++) {
        param_flash_program_half_word(&(dest_buf[i]), src_buf[i]);
    }
    flash_lock();
    if (param_has_entry[key]) {
        squash_needed = true;
    }
    param_has_entry[key] = true;

    return true;
}

static uint16_t param_get_header_data_crc16(struct param_header_data_t* header_data)
{
    return crc16_ccitt((char*)header_data, sizeof(struct param_header_data_t), 0);
}

static uint16_t param_get_tuple_crc16(struct param_tuple_t* tuple)
{
    return crc16_ccitt((char*)tuple, sizeof(struct param_tuple_t), 0);
}

static bool param_flash_program_half_word(uint16_t* addr, uint16_t value)
{
    bool ret;
    // 1. Check that no main Flash memory operation is ongoing by checking the BSY bit in the FLASH_SR register.
    flash_wait_for_last_operation();
    // 2. Set the PG bit in the FLASH_CR register.
    FLASH_CR |= FLASH_CR_PG;
    // 3. Perform the data write (half-word) at the desired address.
    *addr = value;
    // 4. Wait until the BSY bit is reset in the FLASH_SR register.
    flash_wait_for_last_operation();
    // 5. Check the EOP flag in the FLASH_SR register (it is set when the programming operation has succeded), and then clear it by software
    ret = (FLASH_SR & FLASH_SR_EOP) != 0 && (FLASH_SR & (1UL<<2)) == 0 && (FLASH_SR & (1UL<<4)) == 0;
    FLASH_SR &= ~FLASH_SR_EOP;
    // also clear PG for good measure
    FLASH_CR &= ~FLASH_CR_PG;

    return ret;
}

static bool param_flash_erase_page(uint32_t addr)
{
    bool ret;
    // 1. Check that no Flash memory operation is ongoing by checking the BSY bit in the FLASH_CR register.
    flash_wait_for_last_operation();
    // 2. Set the PER bit in the FLASH_CR register
    FLASH_CR |= FLASH_CR_PER;
    // 3. Program the FLASH_AR register to select a page to erase
    FLASH_AR = addr;
    // 4. Set the STRT bit in the FLASH_CR register (see below note)
    FLASH_CR |= FLASH_CR_STRT;
    // 5. Wait for the BSY bit to be reset
    flash_wait_for_last_operation();
    // 6. Check the EOP flag in the FLASH_SR register (it is set when the erase operation has succeded), and then clear it by software.
    ret = (FLASH_SR & FLASH_SR_EOP) != 0;
    FLASH_SR &= ~FLASH_SR_EOP;
    // also clear PER for good measure
    FLASH_CR &= ~FLASH_CR_PER;

    return ret;
}
