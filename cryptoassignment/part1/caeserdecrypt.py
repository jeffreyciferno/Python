#CSE365 Fall 2020
#Jeffrey Ciferno
#jmcifern@asu.edu
#
ciphertext = open("ciphertext.txt","rb").read()
outfile = open("caesarplaintext.txt","w")
plaintext = ''
for i in range(0, len(ciphertext)):
	c = ciphertext[i] - ord('A')
	p = chr(ord('A') + ((c - 22) % 26))
	plaintext = plaintext + p
outfile.write(plaintext)
