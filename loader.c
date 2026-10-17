/******************************************************************************
 * @file            loader.c
 *****************************************************************************/
#include    <stdint.h>
#include    <stdio.h>
#include    <string.h>

#include    "cpu.h"
#include    "loader.h"
#include    "memory.h"

int load_com_file (const char *path, CPU8086 *cpu, uint8_t *mem, size_t mem_size, uint16_t load_seg, size_t *out_size) {

    FILE *fp;
    
    size_t nread;
    size_t max_len;
    
    uint16_t load_addr = (load_seg << 4);
    
    if (out_size != NULL) {
        *out_size = 0;
    }
    
    if (path == NULL || mem == NULL || (size_t) load_addr >= mem_size) {
        return 0;
    }
    
    fp = fopen (path, "rb");
    
    if (fp == NULL) {
        return 0;
    }
    
    max_len = mem_size - (size_t) load_addr;
    memset (mem, 0, mem_size);
    
    nread = fread (mem + load_addr, 1, max_len, fp);
    fclose (fp);
    
    load_addr += nread;
    
    cpu->cs = cpu->ss = load_seg - 0x10;
    cpu->ip = 0x0100;                   /* Code starts at offset 256 */
    
    cpu->regs[REG_SP] = 0xFFFE;         /* Stack at the top */
    
    if (out_size != NULL) {
        *out_size = nread;
    }
    
    return 1;

}

typedef struct __attribute__ ((packed)) {

    uint16_t signature;                 /* Must be 0x5A4D ("MZ") */
    uint16_t extra_bytes;               /* Bytes on last page */
    uint16_t pages;                     /* 512-byte pages in file */
    uint16_t relocations;               /* Number of relocation entries */
    uint16_t header_size;               /* Size of header in 16-byte paragraphs */
    uint16_t min_alloc;                 /* Minimum extra paragraphs needed */
    uint16_t max_alloc;                 /* Maximum extra paragraphs needed */
    uint16_t initial_ss;                /* Initial relative SS */
    uint16_t initial_sp;                /* Initial SP */
    uint16_t checksum;                  /* Usually 0 */
    uint16_t initial_ip;                /* Initial IP */
    uint16_t initial_cs;                /* Initial relative CS */
    uint16_t reloc_table;               /* File offset to relocation table */
    uint16_t overlay_num;               /* Usually 0 */

} MZHeader;

int load_mz_file( const char *path, CPU8086 *cpu, uint16_t base_seg) {

    uint32_t file_size, image_start, total_size, image_size, load_phys;
    uint16_t load_seg, i;
    
    long file_size_long;
    
    FILE *fp;
    MZHeader hdr;
    
    if (!(fp = fopen (path, "rb"))) {
        return 0;
    }
    
    if (fread (&hdr, sizeof (hdr), 1, fp) != 1) {
    
        fclose (fp);
        return 0;
    
    }
    
    if (hdr.signature != 0x5A4D) {
    
        fclose (fp);
        return 0;
    
    }
    
    if (fseek (fp, 0, SEEK_END) != 0) {
    
        fclose (fp);
        return 0;
    
    }
    
    file_size_long = ftell (fp);
    
    if (file_size_long < 0) {
    
        fclose (fp);
        return 0;
    
    }
    
    file_size = (uint32_t) file_size_long;
    image_start = (uint32_t) hdr.header_size * 16;
    
    if (hdr.pages == 0) {
    
        fclose (fp);
        return 0;
    
    }
    
    if (hdr.extra_bytes == 0) {
        total_size = (uint32_t) hdr.pages * 512;
    } else {
        total_size = (uint32_t) (hdr.pages - 1) * 512 + hdr.extra_bytes;
    }
    
    if (total_size < image_start || image_start > file_size) {
    
        fclose (fp);
        return 0;
    
    }
    
    image_size = total_size - image_start;
    
    /**
     * Load image at a single base segment. CS/IP and SS/SP stay relative
     * to that load segment, exactly as defined by the MZ header.
     */
    load_seg = base_seg;
    load_phys = (uint32_t) load_seg << 4;
    
    if (load_phys + image_size > cpu->mem_size) {
    
        fclose (fp);
        return 0;
    
    }
    
    if (fseek (fp, (long) image_start, SEEK_SET) != 0) {
    
        fclose (fp);
        return 0;
    
    }
    
    if (fread (&cpu->mem[load_phys], 1, image_size, fp) != image_size) {
    
        fclose (fp);
        return 0;
    
    }
    
    if (fseek (fp, hdr.reloc_table, SEEK_SET) != 0) {
    
        fclose (fp);
        return 0;
    
    }
    
    for (i = 0; i < hdr.relocations; i++) {
    
        uint16_t offset, segment;
        
        if (fread (&offset, 2, 1, fp) != 1 || fread (&segment, 2, 1, fp) != 1) {
        
            fclose (fp);
            return 0;
        
        }
        
        {
        
            uint32_t fixup_addr = get_phys ((uint16_t) (load_seg + segment), offset);
            uint16_t val;
            
            if (fixup_addr + 1 >= cpu->mem_size) {
            
                fclose (fp);
                return 0;
            
            }
            
            val = mem_read16 (cpu->mem, cpu->mem_size, fixup_addr);
            val = (uint16_t) (val + load_seg);
            
            mem_write16 (cpu->mem, cpu->mem_size, fixup_addr, val);
        
        }
    
    }
    
    cpu->cs = (uint16_t) (load_seg + hdr.initial_cs);
    cpu->ip = hdr.initial_ip;
    
    cpu->ss = (uint16_t) (load_seg + hdr.initial_ss);
    cpu->regs[REG_SP] = hdr.initial_sp;
    
    fclose (fp);
    return 1;
}
