/*
 * Modified by Tane on 2024.
 *
 * Copyright (C) 2016 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BLUEKITCHEN
 * GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#include "Arduino.h"
#include "pico/stdlib.h"
#include "btstack_config.h"
#include "btstack.h"
#include <LittleFS.h>

#include "sbc_encoder.h"
#include "sbc_enc_func_declare.h"
#include "sbc_types.h"
#include "sbc_dct.h"
#include "a2dp_source.h"
#include "btstack_sbc_encoder_bluedroid.c"

#define NUM_CHANNELS 2
#define AUDIO_TIMEOUT_MS 10
#define SBC_STORAGE_SIZE 1030

// device_addr_stringはご自身の環境に合わせて修正して下さい。
// Daiso BT earphone
// static const char *device_addr_string = "41:42:2B:84:12:8D";
// ダイソー Bluetooth スピーカー LBS
static const char *device_addr_string = "FD:94:0B:D6:4D:34";

// 音楽ファイル名。ご自身の環境に合わせて修正して下さい。
// Unsigned 8-bit 48000Hz の WAV ファイルを配置して下さい。
static const char *WAV_FILE_NAME = "/music.wav";

static bd_addr_t device_addr;

static bool scan_active;

// wavファイルの定義
static File wav_file;
static size_t wav_length;
// wavデータはファイルの先頭に４４バイトのヘッダがある。
static const char WAV_START_POINT = 44;
static const int WAV_DATA_BUFFER_SIZE = 1024;
static uint8_t wav_data_buffer[WAV_DATA_BUFFER_SIZE];
static int wav_data_buffer_index = 0;

// static int current_sample_rate = 44100;
static int current_sample_rate = 48000;

static btstack_packet_callback_registration_t hci_event_callback_registration;

static uint8_t media_sbc_codec_configuration[4];

static const int A2DP_SOURCE_DEMO_INQUIRY_DURATION_1280MS = 12;

// A2DPメディア送信に関連する情報を追跡するための構造体変数を宣言しています。この変数は、音楽の送信に関連するさまざまな状態や情報を保持するために使用されます。
// A2DP接続のID、ローカルおよびリモートのストリームエンドポイントID、ストリームの状態、音量など、メディア送信に関する情報を追跡するために使用されます。
// この構造体変数は、A2DPパケットハンドラ関数内でイベントに応じた処理を行う際に参照され、音楽の送信状態を適切に管理するために使用されます。
typedef struct
{
    uint16_t a2dp_cid;     // A2DP接続のID
    uint8_t local_seid;    // ローカルのストリームエンドポイントID
    uint8_t remote_seid;   // リモートのストリームエンドポイントID
    uint8_t stream_opened; // ストリームが開いているかどうかのフラグ
    uint16_t avrcp_cid;

    uint32_t time_audio_data_sent; // ms
    uint32_t acc_num_missed_samples;
    uint32_t samples_ready;
    btstack_timer_source_t audio_timer;
    uint8_t streaming;
    int max_media_payload_size;
    uint32_t rtp_timestamp;

    uint8_t sbc_storage[SBC_STORAGE_SIZE];
    uint16_t sbc_storage_count;
    uint8_t sbc_ready_to_send;

    uint8_t volume; // 音量
} a2dp_media_sending_context_t;
static a2dp_media_sending_context_t media_tracker;

// SBCメディア送信に関連する情報を追跡するための構造体変数を宣言しています。
// この構造体変数は、サンプリング周波数、チャンネルモード、ブロック長、サブバンド数、ビットプール値など、SBCコーデックのさまざまなパラメータを保持します。これらのパラメータは、音声データの圧縮や品質に影響を与えます。
typedef struct
{
    int reconfigure;                                   // 再設定が必要かどうかのフラグ
    int num_channels;                                  // チャンネル数（モノラルまたはステレオ）
    int sampling_frequency;                            // サンプリング周波数（Hz）
    int block_length;                                  // ブロック長
    int subbands;                                      // サブバンド数
    int min_bitpool_value;                             // 最小ビットプール値
    int max_bitpool_value;                             // 最大ビットプール値
    btstack_sbc_channel_mode_t channel_mode;           // チャンネルモード（モノ、デュアル、ステレオ、ジョイントステレオ）
    btstack_sbc_allocation_method_t allocation_method; // 割り当て方法（SNRまたはLOUDNESS）
} media_codec_configuration_sbc_t;
static media_codec_configuration_sbc_t sbc_configuration;

// SBC（Subband Coding）エンコーダの内部状態を保持するための構造体変数です。
// この構造体変数は、エンコーダの内部状態、作業バッファ、およびエンコーディングプロセスに必要なパラメータを保持します。これには、サンプリング周波数、チャンネルモード、ブロック長、サブバンド数、ビットプール値など、SBCエンコーディングの設定が含まれます。
static btstack_sbc_encoder_state_t sbc_encoder_state;

// AVRCP (Audio/Video Remote Control Profile) に関連する再生状態情報を保持するための構造体変数です。
// 再生コマンドを受け取った際には play_info.play_status を再生中に設定し、一時停止コマンドを受け取った際には一時停止中に設定するなどの処理が行われます。また、曲の再生位置の更新や曲の長さの設定も、この構造体を通じて行われます。
typedef struct
{
    uint8_t track_id[8];
    uint32_t song_length_ms;        // 曲の長さ（ミリ秒単位
    avrcp_playback_status_t status; // 再生状態（再生中、一時停止中、停止中など）
    uint32_t song_position_ms;      // 曲の現在位置（ミリ秒単位）0xFFFFFFFF if not supported
} avrcp_play_status_info_t;
static avrcp_play_status_info_t play_info;

static avrcp_track_t track = {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}, 1, "Sample Song", "music.wav", "A2DP Source Demo", "monotone", 12345};

// A2DP (Advanced Audio Distribution Profile) で使用されるSBC (Subband Coding) コーデックの機能を定義しています。この配列は、リモートデバイスに対して、このデバイスがサポートしているSBCコーデックのパラメータを通知するために使用されます。
// この配列は、A2DPのSDPレコードや、AVDTP (Audio/Video Distribution Transport Protocol) のコーデック設定コマンドで使用され、リモートデバイスに対して、このデバイスがサポートしているSBCコーデックの機能を通知するために使用されます。
// Unsigned 8-bit 48000Hz の WAV ファイルを再生する場合の設定です。
static uint8_t media_sbc_codec_capabilities[] = {
    // (AVDTP_SBC_44100 << 4) | AVDTP_SBC_STEREO,
    // (AVDTP_SBC_44100 << 4) | AVDTP_SBC_MONO,
    // (AVDTP_SBC_48000 << 4) | AVDTP_SBC_MONO,
    (AVDTP_SBC_48000 << 4) | AVDTP_SBC_STEREO,
    // 0xFF, //(AVDTP_SBC_BLOCK_LENGTH_16 << 4) | (AVDTP_SBC_SUBBANDS_8 << 2) | AVDTP_SBC_ALLOCATION_METHOD_LOUDNESS,
    (AVDTP_SBC_BLOCK_LENGTH_16 << 4) | (AVDTP_SBC_SUBBANDS_8 << 2) | AVDTP_SBC_ALLOCATION_METHOD_SNR,
    2, // 最小ビットプール値
    53 // 最大ビットプール値
};

static uint8_t sdp_a2dp_source_service_buffer[150];
static uint8_t sdp_avrcp_target_service_buffer[200];
static uint8_t sdp_avrcp_controller_service_buffer[200];
static uint8_t device_id_sdp_service_buffer[100];

//*********ここから関数の宣言*********

static int fs_setup(void);
static int btstack_main(void);

// wav_fileからdata_size分のデータを読み込む
static int read_wav_data(uint8_t *wav_data, int data_size)
{
    if (wav_data_buffer_index >= WAV_DATA_BUFFER_SIZE)
    {
        wav_file.read(wav_data_buffer, WAV_DATA_BUFFER_SIZE);
        wav_data_buffer_index = 0;
        if (wav_file.position() >= wav_length)
        {
            wav_file.close();
            if (fs_setup() == -1)
                return -1;
        }
    }
    memcpy(wav_data, wav_data_buffer + wav_data_buffer_index, data_size);
    wav_data_buffer_index += data_size;
    return 0;
}

// Bluetoothデバイスのスキャン（検出）を開始するためのシンプルな関数です。
// この関数は、Bluetoothデバイスの検出プロセスを開始する際に使用され、周囲のデバイスを検出して接続可能なデバイスのリストを取得するために役立ちます。スキャンが完了すると、検出されたデバイスに関する情報がイベントとして報告され、適切な処理が行われます。
static void a2dp_source_demo_start_scanning(void)
{
    Serial.printf("Start scanning...\n\r");
    // Bluetoothデバイスのスキャンを開始します。
    // A2DP_SOURCE_DEMO_INQUIRY_DURATION_1280MS は、スキャンの持続時間を指定する定数で、1280ミリ秒（約1.28秒）を意味します。
    gap_inquiry_start(A2DP_SOURCE_DEMO_INQUIRY_DURATION_1280MS);
    // スキャンがアクティブな状態になったことを示すフラグを true に設定します。
    scan_active = true;
}

// HCI (Host Controller Interface) イベントを処理するためのパケットハンドラです。主な機能は、Bluetoothデバイスの検出、接続の確立、PINコード要求の処理などを行うことです。
// この関数は、Bluetoothデバイスの検出と接続の処理において重要な役割を果たします。特に、Bluetoothスピーカーなどの特定のデバイスを自動的に検出して接続を試みる機能は、オーディオストリーミングアプリケーションにとって便利です。
static void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    UNUSED(channel);
    UNUSED(size);
    // パケットタイプのチェック:
    // 受信したパケットがHCIイベントパケットであることを確認します。そうでない場合は、処理を終了します。
    if (packet_type != HCI_EVENT_PACKET)
        return;
    uint8_t status;
    UNUSED(status);

    bd_addr_t address;
    uint32_t cod;

    switch (hci_event_packet_get_type(packet))
    {
    case BTSTACK_EVENT_STATE:
        // BTSTACKの状態変更イベント (BTSTACK_EVENT_STATE):
        // BTstackの状態が変更されたことを示します。状態が HCI_STATE_WORKING になった場合、Bluetoothデバイスのスキャンを開始します。
        if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING)
            return;
        a2dp_source_demo_start_scanning();
        break;
    case HCI_EVENT_PIN_CODE_REQUEST:
        // PINコード要求イベント (HCI_EVENT_PIN_CODE_REQUEST):
        // リモートデバイスからPINコードの入力が要求されたことを示します。ここでは、デフォルトのPINコード「0000」を使用して応答します。
        Serial.printf("Pin code request - using '0000'\n\r");
        hci_event_pin_code_request_get_bd_addr(packet, address);
        gap_pin_code_response(address, "0000");
        break;
    case GAP_EVENT_INQUIRY_RESULT:
        // インクワイアリ結果イベント (GAP_EVENT_INQUIRY_RESULT):
        // Bluetoothデバイスが検出されたことを示します。デバイスのアドレス、クラスオブデバイス (CoD)、RSSI、デバイス名などの情報を表示します。検出されたデバイスがBluetoothスピーカーである場合（CoDが一致する場合）、そのデバイスに接続を試みます。
        gap_event_inquiry_result_get_bd_addr(packet, address);
        // print info
        Serial.printf("Device found: %s ", bd_addr_to_str(address));
        cod = gap_event_inquiry_result_get_class_of_device(packet);
        Serial.printf("with COD: %06" PRIx32, cod);
        if (gap_event_inquiry_result_get_rssi_available(packet))
        {
            Serial.printf(", rssi %d dBm", (int8_t)gap_event_inquiry_result_get_rssi(packet));
        }
        if (gap_event_inquiry_result_get_name_available(packet))
        {
            char name_buffer[240];
            int name_len = gap_event_inquiry_result_get_name_len(packet);
            memcpy(name_buffer, gap_event_inquiry_result_get_name(packet), name_len);
            name_buffer[name_len] = 0;
            Serial.printf(", name '%s'", name_buffer);
        }
        Serial.println();
        memcpy(device_addr, address, 6);
        Serial.printf("Bluetooth speaker detected, trying to connect to %s...\n\r", bd_addr_to_str(device_addr));
        scan_active = false;
        gap_inquiry_stop();
        a2dp_source_establish_stream(device_addr, &media_tracker.a2dp_cid);
        break;
    case GAP_EVENT_INQUIRY_COMPLETE:
        // インクワイアリ完了イベント (GAP_EVENT_INQUIRY_COMPLETE):
        // Bluetoothデバイスのスキャンが完了したことを示します。スキャンがアクティブな状態であれば、再びスキャンを開始します。
        if (scan_active)
        {
            Serial.printf("No Bluetooth speakers found, scanning again...\n\r");
            gap_inquiry_start(A2DP_SOURCE_DEMO_INQUIRY_DURATION_1280MS);
        }
        break;
    default:
        break;
    }
}

// AVRCP (Audio/Video Remote Control Profile) コントローラとして動作するデバイスがリモートデバイス（例えば、Bluetoothスピーカーやヘッドセット）からの通知イベントを処理するためのパケットハンドラです。具体的には、音量変更の通知やバッテリーステータスの変更などに対応します。
static void avrcp_controller_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    UNUSED(channel);
    UNUSED(size);

    // パケットタイプのチェック:
    //  受信したパケットがHCIイベントパケットであることを確認します。そうでない場合は、処理を終了します。
    if (packet_type != HCI_EVENT_PACKET)
        return;
    // イベントタイプのチェック:
    // パケットがAVRCPメタイベントであることを確認します。そうでない場合は、処理を終了します。
    if (hci_event_packet_get_type(packet) != HCI_EVENT_AVRCP_META)
        return;
    // AVRCP接続のチェック:
    // AVRCP接続が確立されているかどうかを確認します。接続が確立されていない場合は、処理を終了します。
    if (!media_tracker.avrcp_cid)
        return;

    switch (packet[2])
    {
    case AVRCP_SUBEVENT_NOTIFICATION_VOLUME_CHANGED:
        // 音量変更の通知 (AVRCP_SUBEVENT_NOTIFICATION_VOLUME_CHANGED):
        // リモートデバイスからの音量変更の通知を処理します。絶対音量の値を取得し、パーセンテージとして表示します。
        Serial.printf("AVRCP Controller: Notification Absolute Volume %d %%\n\r", avrcp_subevent_notification_volume_changed_get_absolute_volume(packet) * 100 / 127);
        break;
    case AVRCP_SUBEVENT_NOTIFICATION_EVENT_BATT_STATUS_CHANGED:
        // バッテリーステータスの通知 (AVRCP_SUBEVENT_NOTIFICATION_EVENT_BATT_STATUS_CHANGED):
        // リモートデバイスからのバッテリーステータス変更の通知を処理します。バッテリーステータスの値を取得し、表示します。
        // see avrcp_battery_status_t
        Serial.printf("AVRCP Controller: Notification Battery Status 0x%02x\n\r", avrcp_subevent_notification_event_batt_status_changed_get_battery_status(packet));
        break;
    case AVRCP_SUBEVENT_NOTIFICATION_STATE:
        // 通知状態の通知 (AVRCP_SUBEVENT_NOTIFICATION_STATE):
        // リモートデバイスからの特定のイベントに関する通知の有効/無効状態を処理します。イベントの種類と状態（有効または無効）を表示します。
        Serial.printf("AVRCP Controller: Notification %s - %s\n\r",
                      avrcp_event2str(avrcp_subevent_notification_state_get_event_id(packet)),
                      avrcp_subevent_notification_state_get_enabled(packet) != 0 ? "enabled" : "disabled");
        break;
    default:
        break;
    }
}

// AVRCP (Audio/Video Remote Control Profile) ターゲットとして動作するデバイスがリモートデバイス（例えば、スマートフォンやヘッドセット）からのコマンドを処理するためのパケットハンドラです。具体的には、再生状態の問い合わせや再生コントロール（再生、一時停止、停止など）に対応します。
static void avrcp_target_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    UNUSED(channel);
    UNUSED(size);
    uint8_t status = ERROR_CODE_SUCCESS;

    // パケットタイプのチェック:受信したパケットがHCIイベントパケットであることを確認します。そうでない場合は、処理を終了します。
    if (packet_type != HCI_EVENT_PACKET)
        return;
    // イベントタイプのチェック:パケットがAVRCPメタイベントであることを確認します。そうでない場合は、処理を終了します。
    if (hci_event_packet_get_type(packet) != HCI_EVENT_AVRCP_META)
        return;

    bool button_pressed;
    char const *button_state;
    // avrcp_operation_id_t operation_id;
    uint8_t operation_id;

    switch (packet[2])
    {
    case AVRCP_SUBEVENT_PLAY_STATUS_QUERY:
        // 再生状態の問い合わせ (AVRCP_SUBEVENT_PLAY_STATUS_QUERY):
        // リモートデバイスが現在の再生状態（再生中、一時停止中、停止中など）を問い合わせるイベントです。avrcp_target_play_status 関数を使用して、現在の再生状態をリモートデバイスに返答します。
        status = avrcp_target_play_status(media_tracker.avrcp_cid, play_info.song_length_ms, play_info.song_position_ms, play_info.status);
        break;
    case AVRCP_SUBEVENT_OPERATION:
        // 操作コマンドの処理 (AVRCP_SUBEVENT_OPERATION):
        // リモートデバイスからの操作コマンド（再生、一時停止、停止など）を処理するイベントです。操作IDを取得し、ボタンが押されたかどうかをチェックします。
        // ボタンが押された場合（button_pressed が true）、対応する操作を実行します。例えば、再生ボタンが押された場合は a2dp_source_start_stream 関数を呼び出して再生を開始します。
        operation_id = avrcp_subevent_operation_get_operation_id(packet);
        button_pressed = avrcp_subevent_operation_get_button_pressed(packet) > 0;
        button_state = button_pressed ? "PRESS" : "RELEASE";

        Serial.printf("AVRCP Target: operation %s (%s)\n\r", avrcp_operation2str(operation_id), button_state);

        if (!button_pressed)
        {
            break;
        }
        switch (operation_id)
        {
        case AVRCP_OPERATION_ID_PLAY:
            status = a2dp_source_start_stream(media_tracker.a2dp_cid, media_tracker.local_seid);
            break;
        case AVRCP_OPERATION_ID_PAUSE:
            status = a2dp_source_pause_stream(media_tracker.a2dp_cid, media_tracker.local_seid);
            break;
        case AVRCP_OPERATION_ID_STOP:
            status = a2dp_source_disconnect(media_tracker.a2dp_cid);
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }

    // エラーチェック:
    // 処理中にエラーが発生した場合は、エラーコードと共にログに記録します。
    if (status != ERROR_CODE_SUCCESS)
    {
        Serial.printf("Responding to event 0x%02x failed, status 0x%02x\n\r", packet[2], status);
    }
}

// AVRCP (Audio/Video Remote Control Profile) 関連のイベントを処理するためのパケットハンドラです。主な機能は、AVRCP接続の確立と解放のイベントを処理し、リモートデバイス（例えば、スマートフォンやヘッドセット）からのコントロールコマンドに応答することです。
// リモートデバイスからのコマンドに応答することで、オーディオプレーヤーのリモートコントロールを実現します。
static void avrcp_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    UNUSED(channel);
    UNUSED(size);
    bd_addr_t event_addr;
    uint16_t local_cid;
    uint8_t status = ERROR_CODE_SUCCESS;

    // 1.パケットタイプのチェック:最初に、受信したパケットがHCIイベントパケットであることを確認します。そうでない場合は、処理を終了します。
    if (packet_type != HCI_EVENT_PACKET)
        return;
    // 2.イベントタイプのチェック:次に、パケットがAVRCPメタイベントであることを確認します。そうでない場合は、処理を終了します。
    if (hci_event_packet_get_type(packet) != HCI_EVENT_AVRCP_META)
        return;

    // 3.イベントの処理:switch ステートメントを使用して、パケットのイベントコードに基づいて異なるイベントを処理します。
    switch (packet[2])
    {
    case AVRCP_SUBEVENT_CONNECTION_ESTABLISHED:
        // 4.AVRCP接続確立イベント (AVRCP_SUBEVENT_CONNECTION_ESTABLISHED):AVRCP接続が確立されたことを示します。このイベントでは、接続の状態をチェックし、成功した場合はAVRCP接続IDを記録します。また、リモートデバイスのアドレスを取得し、サポートするイベント通知を設定します（再生状態の変更、トラックの変更など）。
        local_cid = avrcp_subevent_connection_established_get_avrcp_cid(packet);
        status = avrcp_subevent_connection_established_get_status(packet);
        if (status != ERROR_CODE_SUCCESS)
        {
            Serial.printf("AVRCP: Connection failed, local cid 0x%02x, status 0x%02x\n\r", local_cid, status);
            return;
        }
        media_tracker.avrcp_cid = local_cid;
        avrcp_subevent_connection_established_get_bd_addr(packet, event_addr);

        Serial.printf("AVRCP: Channel to %s successfully opened, avrcp_cid 0x%02x\n\r", bd_addr_to_str(event_addr), media_tracker.avrcp_cid);

        avrcp_target_support_event(media_tracker.avrcp_cid, AVRCP_NOTIFICATION_EVENT_PLAYBACK_STATUS_CHANGED);
        avrcp_target_support_event(media_tracker.avrcp_cid, AVRCP_NOTIFICATION_EVENT_TRACK_CHANGED);
        avrcp_target_support_event(media_tracker.avrcp_cid, AVRCP_NOTIFICATION_EVENT_NOW_PLAYING_CONTENT_CHANGED);
        avrcp_target_set_now_playing_info(media_tracker.avrcp_cid, NULL, sizeof(track) / sizeof(avrcp_track_t));

        Serial.printf("Enable Volume Change notification\n\r");
        avrcp_controller_enable_notification(media_tracker.avrcp_cid, AVRCP_NOTIFICATION_EVENT_VOLUME_CHANGED);
        Serial.printf("Enable Battery Status Change notification\n\r");
        avrcp_controller_enable_notification(media_tracker.avrcp_cid, AVRCP_NOTIFICATION_EVENT_BATT_STATUS_CHANGED);
        return;

    case AVRCP_SUBEVENT_CONNECTION_RELEASED:
        // 5.AVRCP接続解放イベント (AVRCP_SUBEVENT_CONNECTION_RELEASED):AVRCP接続が解放されたことを示します。このイベントでは、AVRCP接続IDをクリアし、接続が切断されたことをログに記録します。
        Serial.printf("AVRCP Target: Disconnected, avrcp_cid 0x%02x\n\r", avrcp_subevent_connection_released_get_avrcp_cid(packet));
        media_tracker.avrcp_cid = 0;
        return;
    default:
        break;
    }

    // 6.エラーチェック:イベント処理中にエラーが発生した場合は、エラーコードと共にログに記録します。
    if (status != ERROR_CODE_SUCCESS)
    {
        Serial.printf("Responding to event 0x%02x failed, status 0x%02x\n\r", packet[2], status);
    }
}

// WAVファイルからdata_size分のデータを読み込む処理を実装
// ここでは、ファイル操作関数を使用してデータを読み込む
static int produce_audio(int16_t *pcm_buffer, int data_size)
{
    uint8_t wav_data[data_size];
    if (read_wav_data(wav_data, data_size) == -1)
        return -1;
    for (int count = 0; count < data_size; count++)
    {
        // 正規化:
        // 8ビットのオーディオデータが符号なし整数（例えば、0から255の範囲）である場合、16ビットに拡張する際にデータを正規化する必要があります。これは、8ビットの範囲を16ビットの範囲に合わせるために行われます。
        int16_t scaled_sample = static_cast<int16_t>(wav_data[count] - 0x80) * 256;

        // 左チャンネルと右チャンネルに同じ値を設定
        pcm_buffer[count * 2] = scaled_sample;
        pcm_buffer[count * 2 + 1] = scaled_sample;
    }
    return 0;
}

// オーディオデータをSBC (Subband Coding) 形式にエンコードし、エンコードされたデータを送信用のバッファに格納するための関数です。具体的には、以下の処理を行っています。
//  1.SBCエンコーディングの実行:関数は、PCM (Pulse Code Modulation) 形式のオーディオデータをSBC形式にエンコードします。エンコードは、btstack_sbc_encoder_process_data 関数を使用して行われます。
//  2.オーディオバッファの充填:エンコードされたSBCデータは、context->sbc_storage というバッファに格納されます。このバッファは、Bluetooth経由でリモートデバイスに送信されるためのデータを保持します。
//  3.サンプルの消費:エンコードに使用されたオーディオサンプルの数だけ、context->samples_ready からサンプル数が減算されます。これにより、どれだけのオーディオデータがエンコードされて送信の準備ができているかを追跡します。
//  4.バッファの管理:関数は、エンコードされたデータが最大ペイロードサイズを超えないように、バッファの容量を管理します。バッファがいっぱいになると、送信の準備が整ったとみなされます。
static int a2dp_demo_fill_sbc_audio_buffer(a2dp_media_sending_context_t *context)
{
    // perform sbc encoding
    int total_num_bytes_read = 0;
    unsigned int num_audio_samples_per_sbc_buffer = btstack_sbc_encoder_num_audio_frames();
    while (context->samples_ready >= num_audio_samples_per_sbc_buffer && (context->max_media_payload_size - context->sbc_storage_count) >= btstack_sbc_encoder_sbc_buffer_length())
    {

        int16_t pcm_frame[256 * NUM_CHANNELS];
        if (produce_audio(pcm_frame, num_audio_samples_per_sbc_buffer) == -1)
            return 0;
        // ここでエンコードされる。
        btstack_sbc_encoder_process_data(pcm_frame);

        uint16_t sbc_frame_size = btstack_sbc_encoder_sbc_buffer_length();
        uint8_t *sbc_frame = btstack_sbc_encoder_sbc_buffer();

        total_num_bytes_read += num_audio_samples_per_sbc_buffer;
        // first byte in sbc storage contains sbc media header
        memcpy(&context->sbc_storage[1 + context->sbc_storage_count], sbc_frame, sbc_frame_size);
        context->sbc_storage_count += sbc_frame_size;
        context->samples_ready -= num_audio_samples_per_sbc_buffer;
    }
    return total_num_bytes_read;
}

// A2DPを使用して音声データを定期的に送信するためのタイムアウトハンドラです。
// 関数の役割は、一定の間隔でオーディオデータをエンコードし、送信の準備が整ったら送信リクエストを行うことです。
// この関数は、定期的に呼び出されることで、オーディオデータのエンコードと送信を一定の間隔で行い、安定したオーディオストリーミングを実現します。
static void a2dp_demo_audio_timeout_handler(btstack_timer_source_t *timer)
{
    a2dp_media_sending_context_t *context = (a2dp_media_sending_context_t *)btstack_run_loop_get_timer_context(timer);
    // タイマーの設定。次回のタイムアウトイベントが発生するまでの時間を設定します。AUDIO_TIMEOUT_MS は、タイムアウトの間隔をミリ秒単位で指定します。
    btstack_run_loop_set_timer(&context->audio_timer, AUDIO_TIMEOUT_MS);
    // タイマーの追加。設定したタイマーを実行ループに追加し、タイムアウトイベントの監視を開始します。
    btstack_run_loop_add_timer(&context->audio_timer);
    // 前回オーディオデータが送信されてからの経過時間を計算し、その期間に対応するサンプル数を計算します。これにより、オーディオの再生速度を一定に保つことができます。
    uint32_t now = btstack_run_loop_get_time_ms();

    uint32_t update_period_ms = AUDIO_TIMEOUT_MS;
    if (context->time_audio_data_sent > 0)
    {
        update_period_ms = now - context->time_audio_data_sent;
    }

    uint32_t num_samples = (update_period_ms * current_sample_rate) / 1000;
    context->acc_num_missed_samples += (update_period_ms * current_sample_rate) % 1000;

    while (context->acc_num_missed_samples >= 1000)
    {
        num_samples++;
        context->acc_num_missed_samples -= 1000;
    }
    context->time_audio_data_sent = now;
    context->samples_ready += num_samples;

    if (context->sbc_ready_to_send)
        return;

    // オーディオバッファの充填。
    // オーディオバッファをSBCエンコードされたオーディオデータで充填します。これにより、Bluetooth経由で送信するためのデータが準備されます。
    // この中で、SBC にエンコードしている。
    a2dp_demo_fill_sbc_audio_buffer(context);

    // 送信の準備。
    // 送信するデータが十分に溜まったら（バッファが最大ペイロードサイズを超えたら）、送信リクエストを行います。これにより、リモートデバイスにオーディオデータが送信されます。
    if ((context->sbc_storage_count + btstack_sbc_encoder_sbc_buffer_length()) > context->max_media_payload_size)
    {
        // schedule sending
        context->sbc_ready_to_send = 1;
        a2dp_source_stream_endpoint_request_can_send_now(context->a2dp_cid, context->local_seid);
    }
}

static void a2dp_demo_timer_start(a2dp_media_sending_context_t *context)
{
    context->max_media_payload_size = btstack_min(a2dp_max_media_payload_size(context->a2dp_cid, context->local_seid), SBC_STORAGE_SIZE);
    context->sbc_storage_count = 0;
    context->sbc_ready_to_send = 0;
    context->streaming = 1;
    btstack_run_loop_remove_timer(&context->audio_timer);
    btstack_run_loop_set_timer_handler(&context->audio_timer, a2dp_demo_audio_timeout_handler);
    btstack_run_loop_set_timer_context(&context->audio_timer, context);
    btstack_run_loop_set_timer(&context->audio_timer, AUDIO_TIMEOUT_MS);
    btstack_run_loop_add_timer(&context->audio_timer);
}

static void a2dp_demo_timer_stop(a2dp_media_sending_context_t *context)
{
    context->time_audio_data_sent = 0;
    context->acc_num_missed_samples = 0;
    context->samples_ready = 0;
    context->streaming = 1;
    context->sbc_storage_count = 0;
    context->sbc_ready_to_send = 0;
    btstack_run_loop_remove_timer(&context->audio_timer);
}

// この関数は、A2DP (Advanced Audio Distribution Profile) を使用してSBC (Subband Coding) エンコードされたオーディオデータをBluetooth経由で送信するためのものです。
// この関数は、定期的に呼び出され、エンコード済みのオーディオデータをBluetooth経由でリモートデバイスに送信する役割を果たします。
static void a2dp_demo_send_media_packet(void)
{
    // フレームサイズの計算
    // SBCエンコーダによって生成される各SBCフレームのバイト数を計算します。この値は、エンコーディングプロセスにおいて一定です。
    int num_bytes_in_frame = btstack_sbc_encoder_sbc_buffer_length();
    // ストレージ内のバイト数の計算
    // 現在ストレージに保持されているエンコード済みオーディオデータのバイト数を計算します。
    int bytes_in_storage = media_tracker.sbc_storage_count;
    // SBCフレーム数の計算
    // ストレージに保持されているエンコード済みオーディオデータから生成できるSBCフレームの数を計算します。
    uint8_t num_sbc_frames = bytes_in_storage / num_bytes_in_frame;
    // Prepend SBC Header
    // SBCヘッダの追加
    // SBCフレームの数を最初のバイトに格納して、SBCヘッダを追加します。これは、受信側がどのくらいのフレーム数を受け取るべきかを知るために必要です。
    media_tracker.sbc_storage[0] = num_sbc_frames; // (fragmentation << 7) | (starting_packet << 6) | (last_packet << 5) | num_frames;
    // オーディオデータの送信
    // エンコード済みのオーディオデータ（SBCフレーム）をBluetooth経由で送信します。この関数は、A2DPのストリームエンドポイントID、RTPタイムスタンプ、およびエンコード済みデータを含むSBCストレージを引数として取ります。
    a2dp_source_stream_send_media_payload_rtp(
        media_tracker.a2dp_cid,
        media_tracker.local_seid,
        0,
        media_tracker.rtp_timestamp,
        media_tracker.sbc_storage,
        bytes_in_storage + 1);

    // update rtp_timestamp
    unsigned int num_audio_samples_per_sbc_buffer = btstack_sbc_encoder_num_audio_frames();
    // 次回のオーディオパケットを送信する際に使用するRTPタイムスタンプを更新します。RTPタイムスタンプは、オーディオデータの同期を保つために重要です。
    media_tracker.rtp_timestamp += num_sbc_frames * num_audio_samples_per_sbc_buffer;

    // ストレージと送信フラグのリセット
    // オーディオデータが送信された後にストレージと送信フラグをリセットします。これにより、次のオーディオデータのエンコードと送信の準備が整います。
    media_tracker.sbc_storage_count = 0;
    media_tracker.sbc_ready_to_send = 0;
}

static void dump_sbc_configuration(media_codec_configuration_sbc_t *configuration)
{
    Serial.printf("Received media codec configuration:\n\r");
    Serial.printf("    - num_channels: %d\n\r", configuration->num_channels);
    Serial.printf("    - sampling_frequency: %d\n\r", configuration->sampling_frequency);
    Serial.printf("    - channel_mode: %d\n\r", configuration->channel_mode);
    Serial.printf("    - block_length: %d\n\r", configuration->block_length);
    Serial.printf("    - subbands: %d\n\r", configuration->subbands);
    Serial.printf("    - allocation_method: %d\n\r", configuration->allocation_method);
    Serial.printf("    - bitpool_value [%d, %d] \n\r", configuration->min_bitpool_value, configuration->max_bitpool_value);
}

// A2DP ソースのパケットハンドラ
static void a2dp_source_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    UNUSED(channel);
    UNUSED(size);
    uint8_t status;
    uint8_t local_seid;
    bd_addr_t address;
    uint16_t cid;

    avdtp_channel_mode_t channel_mode;
    uint8_t allocation_method;

    // A2DPに関連するイベントだけを処理します。それ以外のイベントは無視されます。
    if (packet_type != HCI_EVENT_PACKET)
        return;
    if (hci_event_packet_get_type(packet) != HCI_EVENT_A2DP_META)
        return;

    switch (hci_event_a2dp_meta_get_subevent_code(packet))
    {
    case A2DP_SUBEVENT_SIGNALING_CONNECTION_ESTABLISHED:
        // 接続が確立された場合
        // Bluetoothデバイスとの接続が成功したことを示します。このイベントが発生すると、接続の詳細（デバイスのアドレス、接続IDなど）が表示されます。
        if (scan_active)
        {
            gap_inquiry_stop();
            scan_active = false;
        }

        a2dp_subevent_signaling_connection_established_get_bd_addr(packet, address);
        cid = a2dp_subevent_signaling_connection_established_get_a2dp_cid(packet);
        status = a2dp_subevent_signaling_connection_established_get_status(packet);

        if (status != ERROR_CODE_SUCCESS)
        {
            Serial.printf("A2DP Source: Connection failed, status 0x%02x, cid 0x%02x, a2dp_cid 0x%02x \n\r", status, cid, media_tracker.a2dp_cid);
            media_tracker.a2dp_cid = 0;
            break;
        }
        media_tracker.a2dp_cid = cid;
        media_tracker.volume = 10;

        Serial.printf("A2DP Source: Connected to address %s, a2dp cid 0x%02x, local seid 0x%02x.\n\r", bd_addr_to_str(address), media_tracker.a2dp_cid, media_tracker.local_seid);
        break;

    case A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_SBC_CONFIGURATION:
        // メディアコーデックの設定
        // このイベントは、音楽を送信するためのコーデック（圧縮方式）の設定が完了したことを示します。ここで、サンプリング周波数やビットレートなどのパラメータが設定されます。
        {
            cid = avdtp_subevent_signaling_media_codec_sbc_configuration_get_avdtp_cid(packet);
            if (cid != media_tracker.a2dp_cid)
                return;

            media_tracker.remote_seid = a2dp_subevent_signaling_media_codec_sbc_configuration_get_remote_seid(packet);

            sbc_configuration.reconfigure = a2dp_subevent_signaling_media_codec_sbc_configuration_get_reconfigure(packet);
            sbc_configuration.num_channels = a2dp_subevent_signaling_media_codec_sbc_configuration_get_num_channels(packet);
            sbc_configuration.sampling_frequency = a2dp_subevent_signaling_media_codec_sbc_configuration_get_sampling_frequency(packet);
            sbc_configuration.block_length = a2dp_subevent_signaling_media_codec_sbc_configuration_get_block_length(packet);
            sbc_configuration.subbands = a2dp_subevent_signaling_media_codec_sbc_configuration_get_subbands(packet);
            sbc_configuration.min_bitpool_value = a2dp_subevent_signaling_media_codec_sbc_configuration_get_min_bitpool_value(packet);
            sbc_configuration.max_bitpool_value = a2dp_subevent_signaling_media_codec_sbc_configuration_get_max_bitpool_value(packet);

            channel_mode = (avdtp_channel_mode_t)a2dp_subevent_signaling_media_codec_sbc_configuration_get_channel_mode(packet);
            allocation_method = a2dp_subevent_signaling_media_codec_sbc_configuration_get_allocation_method(packet);

            Serial.printf("A2DP Source: Received SBC codec configuration, sampling frequency %u, a2dp_cid 0x%02x, local seid 0x%02x, remote seid 0x%02x.\n\r",
                          sbc_configuration.sampling_frequency, cid,
                          a2dp_subevent_signaling_media_codec_sbc_configuration_get_local_seid(packet),
                          a2dp_subevent_signaling_media_codec_sbc_configuration_get_remote_seid(packet));

            // Adapt Bluetooth spec definition to SBC Encoder expected input
            sbc_configuration.allocation_method = (btstack_sbc_allocation_method_t)(allocation_method - 1);
            switch (channel_mode)
            {
            case AVDTP_CHANNEL_MODE_JOINT_STEREO:
                sbc_configuration.channel_mode = SBC_CHANNEL_MODE_JOINT_STEREO;
                break;
            case AVDTP_CHANNEL_MODE_STEREO:
                sbc_configuration.channel_mode = SBC_CHANNEL_MODE_STEREO;
                break;
            case AVDTP_CHANNEL_MODE_DUAL_CHANNEL:
                sbc_configuration.channel_mode = SBC_CHANNEL_MODE_DUAL_CHANNEL;
                break;
            case AVDTP_CHANNEL_MODE_MONO:
                sbc_configuration.channel_mode = SBC_CHANNEL_MODE_MONO;
                break;
            default:
                btstack_assert(false);
                break;
            }
            dump_sbc_configuration(&sbc_configuration);

            btstack_sbc_encoder_init(&sbc_encoder_state,
                                     SBC_MODE_STANDARD,
                                     sbc_configuration.block_length, sbc_configuration.subbands,
                                     sbc_configuration.allocation_method, sbc_configuration.sampling_frequency,
                                     sbc_configuration.max_bitpool_value,
                                     sbc_configuration.channel_mode);
            break;
        }

    case A2DP_SUBEVENT_STREAM_ESTABLISHED:
        a2dp_subevent_stream_established_get_bd_addr(packet, address);
        status = a2dp_subevent_stream_established_get_status(packet);
        if (status != ERROR_CODE_SUCCESS)
        {
            Serial.printf("A2DP Source: Stream failed, status 0x%02x.\n\r", status);
            break;
        }

        local_seid = a2dp_subevent_stream_established_get_local_seid(packet);
        cid = a2dp_subevent_stream_established_get_a2dp_cid(packet);

        Serial.printf("A2DP Source: Stream established a2dp_cid 0x%02x, local_seid 0x%02x, remote_seid 0x%02x\n\r", cid, local_seid, a2dp_subevent_stream_established_get_remote_seid(packet));

        media_tracker.stream_opened = 1;
        status = a2dp_source_start_stream(media_tracker.a2dp_cid, media_tracker.local_seid);
        break;

    case A2DP_SUBEVENT_STREAM_RECONFIGURED:
        status = a2dp_subevent_stream_reconfigured_get_status(packet);
        local_seid = a2dp_subevent_stream_reconfigured_get_local_seid(packet);
        cid = a2dp_subevent_stream_reconfigured_get_a2dp_cid(packet);

        if (status != ERROR_CODE_SUCCESS)
        {
            Serial.printf("A2DP Source: Stream reconfiguration failed, status 0x%02x\n\r", status);
            break;
        }

        Serial.printf("A2DP Source: Stream reconfigured a2dp_cid 0x%02x, local_seid 0x%02x\n\r", cid, local_seid);
        status = a2dp_source_start_stream(media_tracker.a2dp_cid, media_tracker.local_seid);
        break;

    case A2DP_SUBEVENT_STREAM_STARTED:
        // ストリームが開始された場合
        // 音楽の再生が開始されたことを示します。このイベントが発生すると、音楽を送信する準備が整ったことが示されます。
        local_seid = a2dp_subevent_stream_started_get_local_seid(packet);
        cid = a2dp_subevent_stream_started_get_a2dp_cid(packet);

        play_info.status = AVRCP_PLAYBACK_STATUS_PLAYING;
        if (media_tracker.avrcp_cid)
        {
            avrcp_target_set_now_playing_info(media_tracker.avrcp_cid, &track, sizeof(track) / sizeof(avrcp_track_t));
            avrcp_target_set_playback_status(media_tracker.avrcp_cid, AVRCP_PLAYBACK_STATUS_PLAYING);
        }
        a2dp_demo_timer_start(&media_tracker);
        Serial.printf("A2DP Source: Stream started, a2dp_cid 0x%02x, local_seid 0x%02x\n\r", cid, local_seid);
        break;

    case A2DP_SUBEVENT_STREAMING_CAN_SEND_MEDIA_PACKET_NOW:
        local_seid = a2dp_subevent_streaming_can_send_media_packet_now_get_local_seid(packet);
        cid = a2dp_subevent_signaling_media_codec_sbc_configuration_get_a2dp_cid(packet);
        a2dp_demo_send_media_packet();
        break;

    case A2DP_SUBEVENT_STREAM_SUSPENDED:
        // ストリームが一時停止された場合
        // 音楽の再生が一時停止されたことを示します。
        local_seid = a2dp_subevent_stream_suspended_get_local_seid(packet);
        cid = a2dp_subevent_stream_suspended_get_a2dp_cid(packet);

        play_info.status = AVRCP_PLAYBACK_STATUS_PAUSED;
        if (media_tracker.avrcp_cid)
        {
            avrcp_target_set_playback_status(media_tracker.avrcp_cid, AVRCP_PLAYBACK_STATUS_PAUSED);
        }
        Serial.printf("A2DP Source: Stream paused, a2dp_cid 0x%02x, local_seid 0x%02x\n\r", cid, local_seid);

        a2dp_demo_timer_stop(&media_tracker);
        break;

    case A2DP_SUBEVENT_STREAM_RELEASED:
        // ストリームが解放された場合
        // 音楽の再生が停止され、ストリームが解放されたことを示します。
        play_info.status = AVRCP_PLAYBACK_STATUS_STOPPED;
        cid = a2dp_subevent_stream_released_get_a2dp_cid(packet);
        local_seid = a2dp_subevent_stream_released_get_local_seid(packet);

        Serial.printf("A2DP Source: Stream released, a2dp_cid 0x%02x, local_seid 0x%02x\n\r", cid, local_seid);

        if (cid == media_tracker.a2dp_cid)
        {
            media_tracker.stream_opened = 0;
            Serial.printf("A2DP Source: Stream released.\n\r");
        }
        if (media_tracker.avrcp_cid)
        {
            avrcp_target_set_now_playing_info(media_tracker.avrcp_cid, NULL, sizeof(track) / sizeof(avrcp_track_t));
            avrcp_target_set_playback_status(media_tracker.avrcp_cid, AVRCP_PLAYBACK_STATUS_STOPPED);
        }
        a2dp_demo_timer_stop(&media_tracker);
        break;
    case A2DP_SUBEVENT_SIGNALING_CONNECTION_RELEASED:
        cid = a2dp_subevent_signaling_connection_released_get_a2dp_cid(packet);
        if (cid == media_tracker.a2dp_cid)
        {
            media_tracker.avrcp_cid = 0;
            media_tracker.a2dp_cid = 0;
            Serial.printf("A2DP Source: Signaling released.\n\r\n\r");
        }
        break;
    default:
        break;
    }
}

// HCI イベントハンドラ
static void hci_event_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    // HCI イベントの処理
    switch (hci_event_packet_get_type(packet))
    {
    case HCI_EVENT_CONNECTION_COMPLETE:
        Serial.printf("HCI connection complete\n");
        break;
        // 他のイベントに対する処理
    }
}

// SDPクエリ結果のハンドラ
static void sdp_query_complete_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    // SDPクエリ結果の処理
    // この関数内で、クエリの結果に基づいて次のステップ（例えば、A2DP接続の開始）を行う
    Serial.println("sdp_query_complete_handler");
    if (packet_type == SDP_EVENT_QUERY_COMPLETE)
    {
        Serial.println("SDP query 完了, デバイスとの接続ができました。");
    }
}

static int a2dp_source_and_avrcp_services_init(void)
{
    Serial.println("Bluetoothスタックの初期化");
    // Bluetoothスタックの初期化
    // Request role change on reconnecting headset to always use them in slave mode
    hci_set_master_slave_policy(0);

    // 検出モードを設定して、他のデバイスを見つけやすくしています。
    hci_set_inquiry_mode(INQUIRY_MODE_RSSI_AND_EIR);

    // L2CAPレイヤーを初期化します。これは、Bluetoothのデータ通信に必要な部分です。
    l2cap_init();

    // A2DPソース（音楽を送信する側）を初期化します。
    a2dp_source_init();
    // A2DPソースのパケットハンドラを登録します。
    a2dp_source_register_packet_handler(&a2dp_source_packet_handler);

    // Create stream endpoint
    // ストリームエンドポイントの作成
    // a2dp_source_create_stream_endpoint(...);
    // ストリームエンドポイントを作成します。これは、音楽の送受信に使用されるチャネルのようなものです。
    avdtp_stream_endpoint_t *local_stream_endpoint = a2dp_source_create_stream_endpoint(AVDTP_AUDIO, AVDTP_CODEC_SBC, media_sbc_codec_capabilities, sizeof(media_sbc_codec_capabilities), media_sbc_codec_configuration, sizeof(media_sbc_codec_configuration));
    if (!local_stream_endpoint)
    {
        Serial.printf("A2DP Source: not enough memory to create local stream endpoint\n\r");
        return 1;
    }

    // Store stream enpoint's SEP ID, as it is used by A2DP API to indentify the stream endpoint
    media_tracker.local_seid = avdtp_local_seid(local_stream_endpoint);
    avdtp_source_register_delay_reporting_category(media_tracker.local_seid);

    // Initialize AVRCP Service
    // AVRCPサービス初期化
    // AVRCPサービスを初期化します。
    avrcp_init();
    // AVRCPのパケットハンドラを登録します。
    avrcp_register_packet_handler(&avrcp_packet_handler);
    // Initialize AVRCP Target
    // AVRCPターゲットの初期化
    avrcp_target_init();
    avrcp_target_register_packet_handler(&avrcp_target_packet_handler);

    // Initialize AVRCP Controller
    // AVRCPコントローラーの初期化
    avrcp_controller_init();
    avrcp_controller_register_packet_handler(&avrcp_controller_packet_handler);

    // 【SDP クエリを使用して接続を開始】
    // 文字列のアドレスをバイナリ形式に変換
    sscanf_bd_addr(device_addr_string, device_addr);

    // SDPクエリのセットアップ
    // SDP（Service Discovery Protocol：サービス検出プロトコル）を初期化します。これは、他のデバイスが提供するサービスを検出するために使用されます。
    sdp_init();

    // A2DPソース、AVRCPターゲット、AVRCPコントローラのSDPレコードを登録します。これにより、他のデバイスがこれらのサービスを検出できるようになります。
    // Create A2DP Source service record and register it with SDP
    // A2DPソースのSDPレコードを登録します。
    memset(sdp_a2dp_source_service_buffer, 0, sizeof(sdp_a2dp_source_service_buffer));
    a2dp_source_create_sdp_record(sdp_a2dp_source_service_buffer, 0x10001, AVDTP_SOURCE_FEATURE_MASK_PLAYER, NULL, NULL);
    sdp_register_service(sdp_a2dp_source_service_buffer);

    // Create AVRCP Target service record and register it with SDP. We receive Category 1 commands from the headphone, e.g. play/pause
    // AVRCPターゲットのSDPレコードを登録します。
    memset(sdp_avrcp_target_service_buffer, 0, sizeof(sdp_avrcp_target_service_buffer));
    uint16_t supported_features = AVRCP_FEATURE_MASK_CATEGORY_PLAYER_OR_RECORDER;
    avrcp_target_create_sdp_record(sdp_avrcp_target_service_buffer, 0x10002, supported_features, NULL, NULL);
    sdp_register_service(sdp_avrcp_target_service_buffer);

    // Create AVRCP Controller service record and register it with SDP. We send Category 2 commands to the headphone, e.g. volume up/down
    // AVRCPコントローラーのSDPレコードを登録します。
    memset(sdp_avrcp_controller_service_buffer, 0, sizeof(sdp_avrcp_controller_service_buffer));
    uint16_t controller_supported_features = AVRCP_FEATURE_MASK_CATEGORY_MONITOR_OR_AMPLIFIER;
    avrcp_controller_create_sdp_record(sdp_avrcp_controller_service_buffer, 0x10003, controller_supported_features, NULL, NULL);
    sdp_register_service(sdp_avrcp_controller_service_buffer);

    // Register Device ID (PnP) service SDP record
    // デバイスID（PnP）サービスのSDPレコードを登録します。
    memset(device_id_sdp_service_buffer, 0, sizeof(device_id_sdp_service_buffer));
    device_id_create_sdp_record(device_id_sdp_service_buffer, 0x10004, DEVICE_ID_VENDOR_ID_SOURCE_BLUETOOTH, BLUETOOTH_COMPANY_ID_BLUEKITCHEN_GMBH, 1, 1);
    sdp_register_service(device_id_sdp_service_buffer);

    // Set local name with a template Bluetooth address, that will be automatically
    // replaced with a actual address once it is available, i.e. when BTstack boots
    // up and starts talking to a Bluetooth module.
    // テンプレート Bluetooth アドレスを使用してローカル名を設定します。これは自動的に設定されます。
    // 利用可能になったら、つまり BTstack の起動時に実際のアドレスに置き換えられます
    // 起動して Bluetooth モジュールとの通信を開始します。
    // デバイスのローカル名を設定します。
    gap_set_local_name("A2DP Source 00:00:00:00:00:00");
    // デバイスを検出可能にします。
    gap_discoverable_control(1);
    // デバイスのクラスを設定します。
    gap_set_class_of_device(0x200408);

    // Register for HCI events.
    // HCIイベントハンドラを登録します。
    hci_event_callback_registration.callback = &hci_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    return 0;
}

// BTstackのメイン関数
static int btstack_main(void)
{
    int err = a2dp_source_and_avrcp_services_init();
    if (err)
        return err;

    // Bluetoothデバイスの電源をオンにする
    hci_power_control(HCI_POWER_ON);
    return 0;
}

static int fs_setup()
{
    // audioファイルをオープンする。
    wav_file = LittleFS.open(WAV_FILE_NAME, "r");
    if (!wav_file)
    {
        Serial.println("file open failed");
        return -1;
    }
    wav_length = wav_file.size();
    // 先頭44バイトはヘッダなので読み飛ばす。
    wav_file.seek(WAV_START_POINT, SeekSet);
    // wave_bufferの初期化
    for (int i = 0; i < WAV_DATA_BUFFER_SIZE; i++)
    {
        wav_data_buffer[i] = 0;
    }
    return 0;
}

void setup()
{
    Serial.begin(115200);
    LittleFS.begin();
    if (fs_setup() == -1)
        return;
    Serial.println("start");
    int err = btstack_main();
    if (err)
    {
        Serial.println("btstack_main failed");
    }
}

void loop()
{
}