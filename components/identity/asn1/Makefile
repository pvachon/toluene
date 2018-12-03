ASN1=device_identity.asn

target:
	asn1c -funnamed-unions device_identity.asn
	$(RM) converter-sample.c
	$(RM) Makefile.am.sample

clean:
	$(RM) *.c *.h
	$(RM) *.o

.PHONY: target clean
