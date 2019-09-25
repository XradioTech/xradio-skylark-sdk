#!/bin/bash

imgcfg="$1"
opensslcfg="ca/openssl.cnf"
prikey="ca/cakey.pem"

if [ $# -ne 1 ]; then
	echo "Usage $0 <image_cfg_file>"
	exit 1
fi

if [ ! -e ${imgcfg} ]; then
	echo "${imgcfg} not exist!"
	exit 1
fi

if [ ! -e ${opensslcfg} ]; then
	echo "${opensslcfg} not exist!"
	exit 1
fi

if [ ! -e ${prikey} ]; then
	echo "${prikey} not exist!"
	exit 1
fi

# remove useless keyword and keep the value only
imginfo=`grep bin $imgcfg`
printf "image cfg: $imgcfg\n"
printf "image information:\n$imginfo\n"
imginfo=`echo "$imginfo" | sed "s/id\|\"bin\"\|cert\|flash_offs\|sram_offs\|ep\|attr\|\"\| \|{\|}\|\:\|\t\|\r//g"`

for image in ${imginfo[*]}; do
	imagename=`echo $image  | awk -F "[,]" '{print $2}'`
	cert=`echo $image | awk -F "[,]" '{print $3}'`
	attr=`echo $image  | awk -F "[,]" '{print $7}'`
	printf "imagename $imagename, cert $cert, attr $attr\n"
	if [ "$cert" != "null" -a $[$attr & 0x4] -eq 4 ];then
		printf "generate certificate [\033[31m${cert}\033[0m] for image [\033[31m${imagename}\033[0m]!!\n"
		sha256=`openssl dgst -sha256 $imagename`
		sha256=${sha256: -64}
		printf "update openssl.cnf with image hash '\033[31m$sha256\033[0m'\n"
		sed -i "/^subjectKeyIdentifier=/{s/.*/subjectKeyIdentifier=${sha256}/}" ${opensslcfg}
		openssl req -new -x509 -sha256 -key ${prikey} -outform dcr -out ${cert} -config ${opensslcfg} -days 3650 -batch
		openssl x509 -inform dcr -in $cert -noout -text
#		printf "generate final firmware image: \033[31mxr_system.img\033[0m\n"
#		./mkimage.exe
	fi
done

