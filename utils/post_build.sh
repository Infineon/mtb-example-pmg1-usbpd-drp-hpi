#!/bin/bash
# post build script for combining bootloader and applicaton firmware.
# Script creates bootloader application image (.cyacd2) files.

#set -x
echo off

# param 1 - cymcuelf tool path
export TOOL_PATH=$1

# param 2 - application extension (fw1/fw2).
export APP_EXTN=$2

# param 3 - application file name without extension.
export APP_NAME=$3

# param 4 - Bootloader name
export BOOT_NAME=$4

# param 5 - Total flash row count
export FLASH_ROW_COUNT=$5

# Bootloer last row number
export BOOT_LAST_ROW=$6

# param 6 - SINGLE/DUAL firmware architecture
export DUAL_FIRMWARE=$7

# param 7 - MTB Tools path
export MTB_TOOLS=$8

# param 8 - OS information
export OS_INPUT=$9

# Enable bootloader protectin (enable=1/disable=0)
export BOOT_PROTECTION=1

# Select bin2psochex converter based on OS type
if [ $OS_INPUT = 'Windows_NT' ]
then
	export BIN2PSOCHEX=./bin2psochex.exe
	export SREC_CAT=$MTB_TOOLS/srecord/bin/srec_cat.exe
else 
	OS_NAME=$(uname -s)
	if [ $OS_NAME = 'Linux' ]
	then
		export BIN2PSOCHEX=./bin2psochex_linux
		export SREC_CAT=$MTB_TOOLS/srecord/bin/srec_cat
	else # Default to MAC OS
		export BIN2PSOCHEX=./bin2psochex_mac
		export SREC_CAT=$MTB_TOOLS/srecord/bin/srec_cat
	fi
fi

# Sign an create boot loadable .cyacd2 file 
$TOOL_PATH --sign $APP_NAME'.elf' CRC --output $APP_NAME'_'$APP_EXTN'.elf'
$TOOL_PATH -P $APP_NAME'_'$APP_EXTN'.elf' --output $APP_NAME'_'$APP_EXTN'.cyacd2'

#::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
#Enable write protection for bootloader section
#::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
boot_protection() {
	if [ $BOOT_PROTECTION = 0 ]
	then 
		return
	fi
	pushd utils

	#Create a full image bin file from hex file
	$SREC_CAT $APP_NAME'.hex' -Intel -crop 0x0 0x40000 -o $APP_NAME'.bin' -binary

	#Enable write protection for bootloder flash area
	$BIN2PSOCHEX $1 $BOOT_LAST_ROW $FLASH_ROW_COUNT  $APP_NAME'.bin' $APP_NAME'.hex'
	popd
	
	return
}

#::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
# Merge bootloader and single applicatoin firmware
#::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
merge_single() {
	pushd utils

	#Get device sillicon id from the hex file placed at address offset 0x90500002
	#Binary format(B-Record): Freescale Dragonball bootstrap record format
	#4-byte address (big endian), a 1-byte length, and then data bytes as 
	#indictated by the length
	$SREC_CAT $APP_NAME'.hex' -Intel -crop 0x90500002 0x90500006 -offset -0x90500002 -o sid.bin -B-Record
	read sid<sid.bin
	sid=0x${sid:10:8}
	rm sid.bin
	
	# Remove existing hex and elf binary files.
	rm -f $APP_NAME'.elf' $APP_NAME'.hex'

	# Create composite hex file of Bootloader, FW1
	$TOOL_PATH --merge ../$BOOT_NAME'.elf' $APP_NAME'_fw1.elf' \
	           --output $APP_NAME'.elf' \
	           --hex $APP_NAME'.hex'
				   
	popd
	boot_protection $sid
	return
}

#::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
#Merge bootloader and dual applicatoin firmware
#::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
merge_dual() {
	pushd utils

	#Get device sillicon id from the hex file placed at address offset 0x90500002
	#Binary format(B-Record): Freescale Dragonball bootstrap record format
	#4-byte address (big endian), a 1-byte length, and then data bytes as 
	#indictated by the length
	$SREC_CAT $APP_NAME'.hex' -Intel -crop 0x90500002 0x90500006 -offset -0x90500002 -o sid.bin -B-Record
	read sid<sid.bin
	sid=0x${sid:10:8}
	rm sid.bin
	
    #Remove existing hex and elf binary files.
	rm -f $APP_NAME'.elf' $APP_NAME'.hex'

    #Create composite hex file of Bootloader, FW1 and FW2
	$TOOL_PATH --merge ../$BOOT_NAME'.elf' $APP_NAME'_fw1.elf' $APP_NAME'_fw2.elf' \
	           --output $APP_NAME'.elf' \
		   --hex $APP_NAME'.hex'

	popd
	boot_protection $sid
	return
}

# Build dual FW condition check
if [ $DUAL_FIRMWARE = 'DUAL' ] 
then
	if [ $APP_EXTN = 'fw2' ]
	then
		merge_dual
	fi
else
	merge_single
fi

exit
