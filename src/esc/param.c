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

#define PARAM_FORMAT_VERSION 1
#define PARAM_STORE_SIZE 2048

// The parameter store consists of a format version, followed by the number of parameters stored, followed by N parameter entries, followed by a crc32 of all previous data
struct param_header_s {
    uint16_t format_version;
    uint16_t num_params;
};

struct param_entry_s {
    uint64_t key;
    float val;
};

static uint8_t param_store[PARAM_STORE_SIZE] __attribute__((section(".params")));
static struct param_header_s* const param_header = (struct param_header_s*)&param_store[0];
static struct param_entry_s* const param_entries = (struct param_entry_s*)&param_store[sizeof(struct param_header_s)];

#define MAX_NUM_PARAM_ENTRIES ((PARAM_STORE_SIZE-sizeof(struct param_header_s)-sizeof(uint32_t))/sizeof(struct param_entry_s))

static const struct param_info_s param_info_table[] = {
    {.name = "ESC_ENC_MBIAS",                            .default_val =   0.0f, .min_val = -M_PI_F, .max_val = M_PI_F, .int_val=false},
    {.name = "ESC_ENC_EBIAS",                            .default_val =   0.0f, .min_val = -M_PI_F, .max_val = M_PI_F, .int_val=false},
    {.name = "ESC_MOT_KV",                               .default_val =   0.0f, .min_val =       0, .max_val = 100000, .int_val=false},
    {.name = "ESC_MOT_POLES",                            .default_val =   7.0f, .min_val =       1, .max_val = 100,    .int_val=false},
    {.name = "ESC_MOT_R",                                .default_val =   0.1f, .min_val =       0, .max_val = 100,    .int_val=false},
    {.name = "ESC_MOT_L_D",                              .default_val = 40e-6f, .min_val =       0, .max_val = 1e-3f,  .int_val=false},
    {.name = "ESC_MOT_L_Q",                              .default_val = 70e-6f, .min_val =       0, .max_val = 1e-3f,  .int_val=false},
    {.name = "ESC_FOC_P",                                .default_val =   0.0f, .min_val =       0, .max_val = 30,     .int_val=false},
    {.name = "ESC_FOC_I",                                .default_val =   0.0f, .min_val =       0, .max_val = 30000,  .int_val=false},
    {.name = "uavcan.node_id",                           .default_val =   0.0f, .min_val =       0, .max_val = 127,    .int_val=true},
    {.name = "uavcan.id-uavcan.equipment.esc-esc_index", .default_val =   0.0f, .min_val =       0, .max_val = 32,     .int_val=true},
};

#define NUM_DEFINED_PARAMS (sizeof(param_info_table)/sizeof(*param_info_table))

static uint64_t param_hashes[NUM_DEFINED_PARAMS];
static float param_cache[NUM_DEFINED_PARAMS];

static bool param_store_unwritten(void);
static bool param_check_crc(void);
static int16_t param_get_index_by_hash(uint64_t hash);
static void param_erase_store(void);
static bool param_flash_program_half_word(uint16_t* addr, uint16_t value);
static bool param_flash_erase_page(uint32_t addr);

void param_init(void)
{
    uint16_t i;

    // initialize value cache to default value and pre-compute all hashes
    // TODO: somehow compute hashes at compile-time
    for(i=0; i<NUM_DEFINED_PARAMS; i++) {
        param_cache[i] = param_info_table[i].default_val;
        param_hashes[i] = hash_fnv_1a(strlen(param_info_table[i].name), (uint8_t*)param_info_table[i].name);
    }

    // validate parameter table, erase if invalid
    if (param_header->format_version != PARAM_FORMAT_VERSION || param_header->num_params > MAX_NUM_PARAM_ENTRIES || !param_check_crc()) {
        param_write();
    } else {
        // load parameter values
        for(i=0; i<param_header->num_params; i++) {
            int16_t idx = param_get_index_by_hash(param_entries[i].key);
            if (idx != -1) {
                param_cache[idx] = param_entries[i].val;
            }
        }
    }
}

void param_erase(void)
{
    param_erase_store();

    uint8_t i;
    for(i=0; i<NUM_DEFINED_PARAMS; i++) {
        param_cache[i] = param_info_table[i].default_val;
    }
}

void param_write(void)
{
    if (!param_store_unwritten()) {
        param_erase_store();
    }
    uint16_t param_idx, i;
    uint32_t crc = 0;

    uint16_t* write_addr = (uint16_t*)param_store;

    flash_unlock();

    struct param_header_s new_header = {PARAM_FORMAT_VERSION, 0};
    for (param_idx = 0; param_idx < NUM_DEFINED_PARAMS; param_idx++) {
        if (param_cache[param_idx] != param_info_table[param_idx].default_val) {
            new_header.num_params++;
        }
    }

    for (i = 0; i < sizeof(struct param_header_s)/2; i++) {
        param_flash_program_half_word(write_addr++, ((uint16_t*)&new_header)[i]);
    }
    crc = crc32((uint8_t*)&new_header, sizeof(struct param_header_s), crc);

    for (param_idx=0; param_idx<NUM_DEFINED_PARAMS; param_idx++) {
        if (param_cache[param_idx] != param_info_table[param_idx].default_val) {
            struct param_entry_s new_entry = {param_hashes[param_idx], param_cache[param_idx]};

            for (i=0; i<sizeof(struct param_entry_s)/2; i++) {
                param_flash_program_half_word(write_addr++, ((uint16_t*)&new_entry)[i]);
            }
            crc = crc32((uint8_t*)&new_entry, sizeof(struct param_entry_s), crc);
        }
    }

    for (i=0; i<sizeof(uint32_t)/2; i++) {
        param_flash_program_half_word(write_addr, ((uint16_t*)&crc)[i]);
    }
    flash_lock();
}

uint8_t param_get_num_params(void)
{
    return NUM_DEFINED_PARAMS;
}

float* param_retrieve_by_name(const char* name)
{
    return param_retrieve_by_index(param_get_index_by_name(name));
}

float* param_retrieve_by_index(uint16_t idx)
{
    if (idx >= NUM_DEFINED_PARAMS) {
        return NULL;
    }
    return &param_cache[idx];
}

bool param_index_in_range(uint8_t idx)
{
    return idx < NUM_DEFINED_PARAMS;
}

bool param_get_info_by_index(uint8_t idx, const struct param_info_s** info)
{
    if (idx >= NUM_DEFINED_PARAMS) {
        return false;
    }

    *info = &param_info_table[idx];

    return true;
}

int16_t param_get_index_by_name(const char* name) {
    uint16_t i;
    for(i=0; i<NUM_DEFINED_PARAMS; i++) {
        if(strncmp(name, param_info_table[i].name, PARAM_MAX_NAME_LEN) == 0) {
            return i;
        }
    }
    return -1;
}

static bool param_store_unwritten()
{
    uint32_t i;
    for (i=0; i<PARAM_STORE_SIZE; i++) {
        if (param_store[i] != 0xFF) {
            return false;
        }
    }

    return true;
}

static bool param_check_crc()
{
    size_t crc_data_size = sizeof(struct param_header_s) + sizeof(struct param_entry_s) * param_header->num_params;
    uint32_t crc32_computed;
    uint32_t crc32_provided;
    crc32_computed = crc32(param_store, crc_data_size, 0);
    memcpy(&crc32_provided, &param_store[crc_data_size], sizeof(uint32_t));
    return crc32_computed == crc32_provided;
}

static int16_t param_get_index_by_hash(uint64_t hash)
{
    uint16_t i;
    for (i=0; i<NUM_DEFINED_PARAMS; i++) {
        if (param_hashes[i] == hash) {
            return i;
        }
    }
    return -1;
}

static void param_erase_store(void)
{
    flash_unlock();
    param_flash_erase_page((uint32_t)&param_store);
    flash_lock();
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
