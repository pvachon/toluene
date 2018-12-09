#!/usr/bin/env python3

from pyasn1.codec.der.encoder import encode as der_encoder
from pyasn1.codec.native.decoder import decode

from cryptography.hazmat.backends import default_backend
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.hazmat.primitives.serialization import load_pem_private_key
from cryptography.hazmat.primitives.asymmetric.utils import decode_dss_signature

import device_identity

import argparse
import logging

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

    identity = {
        'wifiESSID' : args.essid,
        'wifiPassword' : args.password,
        'targetHost' : args.host,
        'targetPort' : args.port,
        'deviceId' : args.device_id,
        'identityCertChain' : [],
    }

    di_raw = decode(identity, asn1Spec=device_identity.DeviceIdentityInfo())

    logging.info('Device information: {}'.format(di_raw))

    di = der_encoder(di_raw)

    r, s = decode_dss_signature(pk.sign(di, ec.ECDSA(hashes.SHA256())))

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

if __name__ == '__main__':
    main()

