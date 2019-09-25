#!/usr/bin/python

import sys
import re

map_parse_version = "1.0.4"
print "================================="
print "Usage: "
print "  map_parse_version: " + map_parse_version
print "  map_parse_gcc.py xxx.map"
print "=================================\n"

map_file = sys.argv[1]

ram_section = 0
total_ram = 0
limit_ram = 0

flash_section = 0
total_flash = 0
limit_flash = 0

psram_section = 0
total_psram = 0
limit_psram = 0

map_lines = []
with open(map_file, 'r') as f:
    s = f.read().replace('\r\n', '\n')    
    # find the memory configuration
    mem_config_text = '\n'
    mem_config_text += re.findall('Memory Configuration\n\nName             Origin             Length             Attributes\n([\s\S]+)\nLinker script and memory map', s)[0]
    print "Memory Sections:"
    print mem_config_text

    # find the RAM configuration
    ram_config_text = re.findall('\nRAM +\s+(0x\w+)\s+(0x\w+)\s+xrw\n',mem_config_text)
    ram_config_text += re.findall('\nRAM1 +\s+(0x\w+)\s+(0x\w+)\s+xrw\n',mem_config_text)
    #print ram_config_text
    #print len(ram_config_text)
    if (len(ram_config_text)) == 0:
        print ('no ram definite address hint')
        ram_section = 0
    else:
    # get every RAM configuration's  start - end address
        ram_section = 1
        ram_config = []
        for ram in ram_config_text:
            ram_config += [{'start':int(ram[0], 16), 'end':int(ram[0], 16) + int(ram[1], 16)}]
            limit_ram += int(ram[1], 16)

    # find the FLASH configuration
    flash_config_text = re.findall('FLASH+\s+(0x\w+)\s+(0x\w+)\s+xr\n',mem_config_text)
    #print flash_config_text
    #print len(flash_config_text)
    if (len(flash_config_text)) == 0:
        print ('no flash definite address hint')
        flash_section = 0
    else:
    # get every FLASH configuration's  start - end address
        flash_section = 1
        flash_config = []
        for flash in flash_config_text:
            flash_config += [{'start':int(flash[0], 16), 'end':int(flash[0], 16) + int(flash[1], 16)}]
            limit_flash += int(flash[1], 16)

    # find the PSRAM configuration
    psram_config_text = re.findall('PSRAM+\s+(0x\w+)\s+(0x\w+)\s+xrw\n',mem_config_text)
    #print psram_config_text
    #print len(psram_config_text)
    if (len(psram_config_text)) == 0:
        print ('no psram definite address hint')
        psram_section = 0
    else:
    # get every PSRAM configuration's  start - end address
        psram_section = 1
        psram_config = []
        for psram in psram_config_text:
            psram_config += [{'start':int(psram[0], 16), 'end':int(psram[0], 16) + int(psram[1], 16)}]
            limit_psram += int(psram[1], 16)

    # find memory map (without discard and debug sections)
    mem_map = re.findall('Linker script and memory map([\s\S]+?)OUTPUT\(', s)[0]

    # find sections address - length in memory map
    modules = list(set(item[0] for item in re.findall('0x\w+[\t ]+0x\w+[\t ]+.+?([^/\\\]+\.[ao])(\(.+\.o\))?\n', mem_map)))
    modules.sort(key = lambda x : x.upper())
    modules += ['*fill*']

    for module in modules:
        ram_size = 0
        flash_size = 0
        psram_size = 0
        module = module.replace('+', '\+')
        # get module's sections's address and size
        if(module == '*fill*'):
            sections = map(lambda arg : {'address':int(arg[0], 16), 'size':int(arg[1], 16)}, re.findall('\*fill\*[ \t]+(0x\w+)[ \t]+(0x\w+)[ \t]+\n', mem_map))
        else:
            sections = map(lambda arg : {'address':int(arg[0], 16), 'size':int(arg[1], 16)}, re.findall('(0x\w+)[ \t]+(0x\w+)[ \t]+.+[/\\\]'+module+'(\(.+\.o\))?\n', mem_map))
        if(not sections):
            continue

        if ram_section == 1:
	        def ram_size(arg):
	            for ram_info in ram_config:
	                if(ram_info['start'] < arg['address'] < ram_info['end']):
	                    return arg['size']
	            return 0

        if flash_section == 1:
	        def flash_size(arg):
	            for flash_info in flash_config:
	                if(flash_info['start'] < arg['address'] < flash_info['end']):
	                    return arg['size']
	            return 0

        if psram_section == 1:
	        def psram_size(arg):
	            for psram_info in psram_config:
	                if(psram_info['start'] < arg['address'] < psram_info['end']):
	                    return arg['size']
	            return 0

        if ram_section == 1:
            ram_size = reduce(lambda x,y:x+y, map(ram_size, sections))
            total_ram += ram_size
        if flash_section == 1:
            flash_size = reduce(lambda x,y:x+y, map(flash_size, sections))
            total_flash += flash_size
        if psram_section == 1:
            psram_size = reduce(lambda x,y:x+y, map(psram_size, sections))
            total_psram += psram_size

        map_lines.append('| %-40s | %-8d | %-8d | %-8d |'%(re.sub('\.[ao]','',module)[:40],ram_size, flash_size, psram_size))

print '\n                            MEMORY MAP                            '
print '|===========================================================================|'
print '| %-40s | %-8s | %-8s | %-8s |'%('MODULE','RAM', 'FLASH', 'PSRAM')
print '|===========================================================================|'
for line in map_lines:
    print line
print '|===========================================================================|'
print '| %-40s | %-8d | %-8d | %-8d |'%('TOTAL (bytes)', total_ram, total_flash, total_psram)
print '| %-40s | %-8d | %-8d | %-8d |'%('MEM LIMIT', limit_ram, limit_flash, limit_psram)
print '| %-40s | %-8d | %-8d | %-8d |'%('MEM LEFT', limit_ram-total_ram, limit_flash-total_flash, limit_psram-total_psram)
print '|===========================================================================|'

