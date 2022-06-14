




import os
import time
#import socket
import sys
import string
ciphertext = open("juliaplaintext.txt.gz.enc","rb").read()
outfile = open("juliaplaintext.txt","w")
plaintext = []
keyxor = []
enc_char = ''
key="ddqlvuGHUwaO"
keyxorasstring = ''
keybytes= []
#def bit_rotate(this,that):
#	this = (this >> 8-that)
	
#return rotated

keybytes = str.encode(key)
keyshift = keybytes[0] % 7 + 1

def lrot(n, d): return ((n >> d)|(n << (8 - d)))&0xff

for i in range(0, 12):
	keyxor.append(ord(string.ascii_letters[keybytes[i] % len(string.ascii_letters)]))
	keyxorasstring = keyxorasstring + chr(keyxor[i-1])
	print('Key is shifting by ' + str(keyshift) + ' and XORing with ' + keyxorasstring)

#for i in range(0, len(key)):
#        key = int(i)
#        keyxor.append((string.ascii_letters[key % len(string.ascii_letters)]))
#        keyxor = keyxor + string.ascii_letters[int(key) % len(string.ascii_letters)]
#        keyxorasstring = keyxorasstring + chr(int(keyxor[i-1]))
#        print('key' + keyxorastring)
for i in range(0, len(ciphertext)):
	plaintext.append(keyxor[i % len(keyxor)] ^ lrot(ciphertext[i], keyshift))
	print(plaintext[i])
	enc_char = enc_char+ chr(int(i))
#	print('this char' + enc_char)
	#enc_char[i] = bit_rotate(ciphertext[i], 2) #^ bytes(key[j],"utf8")
#	enc_char[i] = int(enc_char[i])
	#print((ciphertext))
	#if len(key) < len(ciphertext) or j == (len(key) - 1) :
		
	#	j = 0
	#else:
	#	j = j + 1

	outfile.write(enc_char)  
  
                       
