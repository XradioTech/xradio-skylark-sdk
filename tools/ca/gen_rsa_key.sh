#!/bin/bash

keyname=cakey

# generate 1024-bit RSA private key
openssl genrsa -out ${keyname}.pem 1024

# generate RSA public key (pem and dcr format) from RSA private key
openssl rsa -in ${keyname}.pem -pubout -out ${keyname}_pub.pem
openssl rsa -in ${keyname}.pem -pubout -outform dcr -out ${keyname}_pub.dcr

# generate SHA256 hash value for RSA public key (dcr format)
openssl dgst -sha256 ${keyname}_pub.dcr > ${keyname}_pub_dcr_hash.txt