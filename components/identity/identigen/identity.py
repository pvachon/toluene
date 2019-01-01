#!/usr/bin/env python3

from pyasn1.codec.der.encoder import encode as der_encoder
from pyasn1.codec.native.decoder import decode

from cryptography.hazmat.backends import default_backend
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.hazmat.primitives.serialization import load_pem_private_key
from cryptography.hazmat.primitives.asymmetric.utils import decode_dss_signature

from cryptography.x509 import load_pem_x509_csr, load_pem_x509_certificate
import cryptography.x509 as x509

import device_identity

import argparse
import datetime
import logging
import serial
import struct
import io
import hexdump

def uart_read_csr(ser):
    sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser), newline=None, errors='replace')
    csr = ''

    in_csr = False

    while True:
        l = sio.readline()

        l = l.strip()

        if l:
            print('{}'.format(l))

            if 'BEGIN CERTIFICATE REQUEST' in l:
                logging.info('Started grabbing CSR')
                in_csr = True

            if True == in_csr:
                csr += l
                csr += '\n'

            if 'END CERTIFICATE REQUEST' in l:
                logging.info('Got CSR, breaking')
                return csr.encode('utf-8')

def uart_write_identity(ser, blob):
    header = struct.pack('<LH', 0xbebafeca, len(blob))
    ser.write(header)
    ser.write(blob)

    sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser), newline=None)

    while True:
        l = sio.readline().strip()
        if l:
            print('{}'.format(l))

def generate_identity_certificate(csr_pem, ca_signing_key, ca_certificates, valid_days):
    # Load the CSR
    csr = load_pem_x509_csr(csr_pem, default_backend())

    # TODO: check the CSR parameters
    signing_cert = ca_certificates[0]

    time_now = datetime.datetime.utcnow()

    # Generate a certificate from the CSR using the X.509 Certificate Builder
    cert = x509.CertificateBuilder().subject_name(
            csr.subject
        ).issuer_name(
            signing_cert.subject
        ).public_key(
            csr.public_key()
        ).serial_number(
            x509.random_serial_number()
        ).not_valid_before(
            time_now
        ).not_valid_after(
            time_now + datetime.timedelta(days=valid_days)
        ).sign(ca_signing_key, hashes.SHA256(), default_backend())

    # Return the entire chain, DER encoded
    chain = [ cert ] + ca_certificates

    return [ x.public_bytes(serialization.Encoding.DER) for x in chain ]


def main():
    parser = argparse.ArgumentParser(description='Prepare a device identity blob for a Toluene device')
    parser.add_argument('-v', '--verbose', help='verbose output', action='store_true')
    parser.add_argument('-E', '--wifi-essid', help='access point ESSID', required=True, dest='essid')
    parser.add_argument('-P', '--wifi-password', help='access point password', required=True, dest='password')
    parser.add_argument('-H', '--service-host', help='host name for the service host', required=True, dest='host')
    parser.add_argument('-p', '--service-port', help='service port number to connect to', required=True, dest='port')
    parser.add_argument('-I', '--device-id', help='device identifier used during submission', required=True, type=lambda x: int(x, 0))
    parser.add_argument('-K', '--private-key', help='private key file name', required=True, dest='private_key')
    parser.add_argument('-o', '--output-file', help='output file', dest='out_file', default='')
    parser.add_argument('-U', '--uart', help='UART to write the identity blob out to', dest='uart', default='')
    parser.add_argument('-B', '--baud', help='The baud rate', dest='baud_rate', default=115200)
    parser.add_argument('-C', '--ca-signing-key', help='The CA signing private key', required=True)
    parser.add_argument('-c', '--ca-chain', help='The certificate chain that attests the CA signing key (as a PEM file)', required=True)
    parser.add_argument('-L', '--valid-length', help='The length of time (in days) the certificate is valid for, starting from this moment', required=False, default=1024, type=int)

    args = parser.parse_args()

    # Set verbosity, globally.
    if args.verbose:
        log_level = logging.DEBUG
    else:
        log_level = logging.INFO

    logging.basicConfig(format='%(asctime)s - %(name)s:%(levelname)s:%(message)s', datefmt='%m/%d/%Y %H:%M:%S', level=log_level)

    logging.info('Loading key from file {}'.format(args.private_key))

    # Read in the key
    with open(args.private_key, 'rb') as fp:
        raw_key = fp.read()

    pk = load_pem_private_key(raw_key, password=None, backend=default_backend())

    if args.uart:
        logging.info('Using UART {} ({},8n1)'.format(args.uart, args.baud_rate))
        ser = serial.Serial(args.uart, args.baud_rate, timeout=1, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, rtscts=0)

        logging.info('Reading CSR from device...')
        csr = uart_read_csr(ser)
        logging.debug('Got csr  {}'.format(csr))

    # Load the CA signing key
    with open(args.ca_signing_key, 'rb') as fp:
        ca_signing_key_pem = fp.read()

    ca_signing_key = load_pem_private_key(ca_signing_key_pem, password=None, backend=default_backend())

    # Load the CA chain of certificates
    logging.debug('Loading CA chain')

    with open(args.ca_chain, 'rb') as fp:
        ca_certificates = []
        cur_crt = b''
        for line in fp:
            if b'BEGIN' in line:
                logging.debug('Starting new certificate')
                cur_crt = line
            elif b'END' in line:
                cur_crt += line
                logging.debug('Parsing certificate')
                if cur_crt:
                    ca_certificates.append(load_pem_x509_certificate(cur_crt, default_backend()))
                cur_crt = ''
            else:
                cur_crt += line

    ident_certs = generate_identity_certificate(csr, ca_signing_key, ca_certificates, args.valid_length)

    identity = {
        'wifiESSID' : args.essid,
        'wifiPassword' : args.password,
        'targetHost' : args.host,
        'targetPort' : args.port,
        'deviceId' : args.device_id,
        'serverCaRoot' : b'foobarbaz',
        'identityCertChain' : ident_certs,
    }

    di_raw = decode(identity, asn1Spec=device_identity.DeviceIdentityInfo())

    logging.info('Device information: {}'.format(di_raw))

    di = der_encoder(di_raw)

    hexdump.hexdump(di)
    r, s = decode_dss_signature(pk.sign(di, ec.ECDSA(hashes.SHA256())))
    hashctx = hashes.Hash(hashes.SHA256(), default_backend())
    hashctx.update(di)
    logging.info('Digest:')
    hexdump.hexdump(hashctx.finalize())

    logging.debug('Device Identity Blob length: {}'.format(len(di)))
    logging.debug('Signature = ({}, {})'.format(r, s))

    bundle = {
        'identityInfo': di_raw,
        'identityInfoSignature': {
            'r': r,
            's': s,
        },
    }
    bundle_raw = decode(bundle, asn1Spec=device_identity.DeviceIdentityBundle())
    bundle_encoded = der_encoder(bundle_raw)

    if args.out_file:
        logging.debug('Writing to {}'.format(args.out_file))
        with open(args.out_file, 'wb+') as fp:
            fp.write(bundle_encoded)

    if args.uart:
        ser = serial.Serial(args.uart, args.baud_rate, timeout=1, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, rtscts=0)
        logging.info('Writing identity to device')
        uart_write_identity(ser, bundle_encoded)

if __name__ == '__main__':
    main()

