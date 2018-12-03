#pragma once

#include <stdint.h>
#include <stddef.h>

#define IDENTITY_BLOB_LENGTH_MAX            4096

/**
 * Structure representing the decoded and verified identity for this device.
 */
struct identity {
    /**
     * The 802.11 AP ESSID we are to connect to
     */
    char wifi_essid[64];

    /**
     * The 802.11 AP's pre-shared key (for WPA2)
     */
    char wifi_password[64];

    /**
     * The target host name, to be resolved using DNS
     */
    char target_host[128];

    /**
     * Then port on the target host
     */
    uint16_t target_port;

    /**
     * The device identifier, to be included in messages to the backend
     */
    uint32_t device_id;
};

/**
 * Load the device's identity, and read it into an easy-to-use structure after
 * validating the signature and other important details. This is represented by
 * the ASN.1 compound structure DeviceIdentityBundle.
 *
 * \param pident The identity struct to be populated (see above)
 * \param bundle The raw bundle, encoded as ASN.1 DER
 * \param bundle_length The length of the bundle, in bytes
 *
 * \return 0 on success, an error code otherwise.
 */
int identity_read(struct identity *pident, void const *bundle, size_t bundle_length);

