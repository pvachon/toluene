#include "identity.h"

#include "DeviceIdentityBundle.h"
#include "DeviceIdentityInfo.h"
#include "ber_decoder.h"

#include "esp_log.h"

#define TAG                         "IDENT"

/**
 * Decode the device identity blob structure, and verify the provided signature
 */
static
int _identity_decode_asn1_bundle(void const *blob, size_t blob_len, DeviceIdentityBundle_t *bndl)
{
    int ret = -1;

    asn_dec_rval_t dec_ret;

    if (NULL == bndl) {
        ESP_LOGE(TAG, "Programmer error: NULL reference to device identity bundle provided.");
        abort();
    }

    dec_ret = ber_decode(NULL, &asn_DEF_DeviceIdentityBundle, (void **)&bndl, blob, blob_len);
    if (dec_ret.code != RC_OK) {
        ESP_LOGE(TAG, "Bad DeviceIdentityBundle provided, aborting (reason: %d, consumed %zu).", dec_ret.code, dec_ret.consumed);
        goto done;
    }

    if (bndl->identityInfo.size <= 0) {
        ESP_LOGE(TAG, "Definitely malformed identity bundle, aborting.");
        goto done;
    }

    ret = 0;
done:
    if (0 != ret) {
        if (NULL !=  bndl) {
            ASN_STRUCT_FREE_CONTENTS_ONLY(asn_DEF_DeviceIdentityBundle, bndl);
            bndl = NULL;
        }
    }

    return ret;
}

/**
 * Decode the device identity info structure
 */
static
int _identity_decode_asn1_info(void const *blob, size_t blob_len, DeviceIdentityInfo_t *info)
{
    int ret = -1;

    asn_dec_rval_t dec_ret;

    if (NULL == info) {
        ESP_LOGE(TAG, "Programmer error: NULL reference to device identity info provided.");
        abort();
    }

    dec_ret = ber_decode(NULL, &asn_DEF_DeviceIdentityInfo, (void **)&info, blob, blob_len);
    if (dec_ret.code != RC_OK) {
        ESP_LOGE(TAG, "Bad DeviceIdentityInfo provided, aborting (reason: %d, consumed %zu).", dec_ret.code, dec_ret.consumed);
        goto done;
    }

    ret = 0;
done:
    if (0 != ret) {
        if (NULL !=  info) {
            ASN_STRUCT_FREE_CONTENTS_ONLY(asn_DEF_DeviceIdentityInfo, info);
            info = NULL;
        }
    }
    return ret;
}

static
int _identity_verify_info_signature(DeviceIdentityBundle_t *bndl, bool *psig_result)
{
    int ret = -1;

    if (NULL == psig_result || NULL == bndl) {
        ESP_LOGE(TAG, "Invalid arguments, aborting.");
        goto done;
    }

    *psig_result = false;

    ESP_LOGW(TAG, "Signature validation is currently disabled, skipping.");

    *psig_result = true;

    ret = 0;
done:
    return ret;
}

int identity_read(struct identity *pident, void const *bundle, size_t bundle_length)
{
    int ret = -1;

    DeviceIdentityBundle_t bndl;
    DeviceIdentityInfo_t info;
    bool sig_result = false;

    if (NULL == bundle || 0 == bundle_length) {
        ESP_LOGE(TAG, "Malformed device identity bundle provided, aborting.");
        goto done;
    }

    if (NULL == pident) {
        ESP_LOGE(TAG, "Malformed target configuration structure, aborting.");
        goto done;
    }

    memset(&bndl, 0, sizeof(bndl));
    memset(&info, 0, sizeof(info));

    if (_identity_decode_asn1_bundle(bundle, bundle_length, &bndl)) {
        ESP_LOGE(TAG, "Device identity bundle is malformed, aborting.");
        goto done;
    }

    /* Verify the signature on the blob */
    if (_identity_verify_info_signature(&bndl, &sig_result)) {
        ESP_LOGE(TAG, "Fatal: error checking signature for bundle, aborting.");
        goto done;
    }

    if (false == sig_result) {
        ESP_LOGE(TAG, "Fatal: incorrect signature on identity info bundle, aborting.");
        goto done;
    }

    /* Parse the info bundle */
    if (_identity_decode_asn1_info(bndl.identityInfo.buf, bndl.identityInfo.size, &info)) {
        ESP_LOGE(TAG, "Fatal: identity info decoding failed, aborting.");
        goto done;
    }

    /* TODO: verify the identity certificate chain, to make sure it is valid */


    if (info.wifiESSID.size > 63 || info.wifiPassword.size > 63 || info.targetHost.size > 127) {
        ESP_LOGE(TAG, "Fatal: ESSID, Password or Target Host information is too long, aborting.");
        goto done;
    }

    memset(pident, 0, sizeof(struct identity));

    /* Populate the identity structure */
    pident->target_port = info.targetPort;
    pident->device_id = info.deviceId;

    memcpy(pident->wifi_essid, info.wifiESSID.buf, info.wifiESSID.size);
    memcpy(pident->wifi_password, info.wifiPassword.buf, info.wifiPassword.size);
    memcpy(pident->target_host, info.targetHost.buf, info.targetHost.size);

    ret = 0;
done:
    ASN_STRUCT_FREE_CONTENTS_ONLY(asn_DEF_DeviceIdentityInfo, &info);
    ASN_STRUCT_FREE_CONTENTS_ONLY(asn_DEF_DeviceIdentityBundle, &bndl);

    return ret;
}

