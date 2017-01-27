/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <esc/param.h>
#include <esc/helpers.h>
#include <esc/semihost_debug.h>
#include <esc/timing.h>

#include <float.h>
#include <string.h>
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



static const struct param_info_s param_info_table[] = {
    {.key = PARAM_ESC_ENC_MBIAS,  .name = "ESC_ENC_MBIAS",                            .default_val =   0.0f, .min_val = -M_PI_F, .max_val = M_PI_F, .int_val=false},
    {.key = PARAM_ESC_ENC_EBIAS,  .name = "ESC_ENC_EBIAS",                            .default_val =   0.0f, .min_val = -M_PI_F, .max_val = M_PI_F, .int_val=false},
    {.key = PARAM_ESC_MOT_KV,     .name = "ESC_MOT_KV",                               .default_val =   0.0f, .min_val =       0, .max_val = 100000, .int_val=false},
    {.key = PARAM_ESC_MOT_POLES,  .name = "ESC_MOT_POLES",                            .default_val =   7.0f, .min_val =       1, .max_val = 100,    .int_val=false},
    {.key = PARAM_ESC_MOT_R,      .name = "ESC_MOT_R",                                .default_val =   0.1f, .min_val =       0, .max_val = 100,    .int_val=false},
    {.key = PARAM_ESC_MOT_L_D,    .name = "ESC_MOT_L_D",                              .default_val = 40e-6f, .min_val =       0, .max_val = 1e-3f,  .int_val=false},
    {.key = PARAM_ESC_MOT_L_Q,    .name = "ESC_MOT_L_Q",                              .default_val = 70e-6f, .min_val =       0, .max_val = 1e-3f,  .int_val=false},
    {.key = PARAM_ESC_FOC_P,      .name = "ESC_FOC_P",                                .default_val =   0.0f, .min_val =       0, .max_val = 30,     .int_val=false},
    {.key = PARAM_ESC_FOC_I,      .name = "ESC_FOC_I",                                .default_val =   0.0f, .min_val =       0, .max_val = 30000,  .int_val=false},
    {.key = PARAM_UAVCAN_NODE_ID, .name = "uavcan.node_id",                           .default_val =   0.0f, .min_val =       0, .max_val = 127,    .int_val=true},
    {.key = PARAM_UAVCAN_ESC_ID,  .name = "uavcan.id-uavcan.equipment.esc-esc_index", .default_val =   0.0f, .min_val =       0, .max_val = 32,     .int_val=true},
};

#define NUM_PARAMS (sizeof(param_info_table)/sizeof(*param_info_table))

static struct param_page_t page_A __attribute__((section(".param_page_a")));
static struct param_page_t page_B __attribute__((section(".param_page_b")));
static struct param_page_t* page_in_use = NULL;
static bool squash_needed = false;
static float param_cache[NUM_PARAMS];
static uint8_t param_has_entry[NUM_PARAMS];

static struct param_page_t* param_get_most_recent_valid_page(void);
static bool param_append_to_page(const struct param_page_t* page, enum param_key_t key, float value);
static bool param_flash_program_half_word(uint16_t* addr, uint16_t value);
static bool param_flash_erase_page(uint32_t addr);
static uint16_t param_get_header_data_crc16(struct param_header_data_t* header_data);
static uint16_t param_get_tuple_crc16(struct param_tuple_t* tuple);
static int16_t param_get_index(enum param_key_t key);

void param_erase(void)
{
    flash_unlock();
    param_flash_erase_page((uint32_t)&page_A);
    param_flash_erase_page((uint32_t)&page_B);
    flash_lock();

    uint8_t i;
    for(i=0; i<NUM_PARAMS; i++) {
        param_cache[i] = param_info_table[i].default_val;
    }
}

void param_init(void)
{
    uint8_t i;
    for(i=0; i<NUM_PARAMS; i++) {
        param_cache[i] = param_info_table[i].default_val;
    }
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

        int16_t param_idx = param_get_index(page_in_use->journal[i].data.key);

        if (param_idx != -1 && page_in_use->journal[i].data_crc16 == param_get_tuple_crc16(&(page_in_use->journal[i].data))) {
            param_cache[param_idx] = page_in_use->journal[i].data.value;
            if (param_has_entry[param_idx]) {
                squash_needed = true;
            }
            param_has_entry[param_idx] = true;
        } else {
            squash_needed = true;
        }
    }

    // squash the journal if required
    param_squash();
}

uint8_t param_get_num_params(void)
{
    return NUM_PARAMS;
}

float* param_retrieve_by_key(enum param_key_t key)
{
    return param_retrieve_by_index(param_get_index(key));
}

float* param_retrieve_by_index(uint16_t idx)
{
    if (idx >= NUM_PARAMS) {
        return NULL;
    }
    return &param_cache[idx];
}

bool param_index_in_range(uint8_t idx)
{
    return idx < NUM_PARAMS;
}

bool param_get_info_by_index(uint8_t idx, const struct param_info_s** info)
{
    if (idx >= NUM_PARAMS) {
        return false;
    }

    *info = &param_info_table[idx];

    return true;
}

int16_t param_get_index_by_name(char* name) {
    uint16_t i;
    for(i=0; i<NUM_PARAMS; i++) {
        if(strncmp(name, param_info_table[i].name, PARAM_MAX_NAME_LEN) == 0) {
            return i;
        }
    }
    return -1;
}

bool param_set_and_save_by_index(uint16_t idx, float value)
{
    if (idx >= NUM_PARAMS) {
        return false;
    }
    return param_set_and_save_by_key(param_info_table[idx].key, value);
}

bool param_set_and_save_by_key(enum param_key_t key, float value)
{
    int16_t param_idx = param_get_index(key);
    if (page_in_use == NULL || param_idx == -1) {
        return false;
    }

    bool success;
    success = param_append_to_page(page_in_use, key, value);
    if (!success && squash_needed) {
        param_squash();
        success = param_append_to_page(page_in_use, key, value);
    }

    if(success) {
        param_cache[param_idx] = value;
    }

    return success;
}

static struct param_page_t* param_get_most_recent_valid_page(void) {
    volatile uint16_t page_A_crc16_computed = param_get_header_data_crc16(&page_A.header.data);
    volatile uint16_t page_B_crc16_computed = param_get_header_data_crc16(&page_B.header.data);

    bool page_A_valid = page_A.header.data.format_version == PARAM_FORMAT_VERSION && page_A.header.data_crc16 == page_A_crc16_computed;
    bool page_B_valid = page_B.header.data.format_version == PARAM_FORMAT_VERSION && page_B.header.data_crc16 == page_B_crc16_computed;

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
        int16_t k = param_info_table[i].key;
        if(param_has_entry[k]) {
            param_append_to_page(next_page, k, param_cache[k]);
        }
    }

    flash_unlock();
    param_flash_program_half_word(&next_page->header.data.format_version, PARAM_FORMAT_VERSION);
    param_flash_program_half_word(&next_page->header.data.seq, (uint16_t)(page_in_use->header.data.seq+1));
    param_flash_program_half_word(&next_page->header.data_crc16, param_get_header_data_crc16(&next_page->header.data));
    flash_lock();

    page_in_use = next_page;
}

static bool param_append_to_page(const struct param_page_t* page, enum param_key_t key, float value)
{
    uint8_t i=0;
    int16_t param_idx = param_get_index(key);

    if (param_idx == -1) {
        return false;
    }

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
        already_set = page->journal[i].data.key == entry.data.key && page->journal[i].data.value == entry.data.value && page->journal[i].data_crc16 == entry.data_crc16;
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

    if (param_has_entry[param_idx]) {
        squash_needed = true;
    }
    param_has_entry[param_idx] = true;

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

static bool __attribute__ ((noinline)) param_flash_program_half_word(uint16_t* addr, uint16_t value)
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

    flash_wait_for_last_operation();

    return ret;
}

static bool __attribute__ ((noinline)) param_flash_erase_page(uint32_t addr)
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

static int16_t param_get_index(enum param_key_t key)
{
    uint16_t i;
    for(i=0; i<NUM_PARAMS; i++) {
        if(param_info_table[i].key == key) {
            return i;
        }
    }
    return -1;
}
