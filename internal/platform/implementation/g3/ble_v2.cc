// Copyright 2022 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "internal/platform/implementation/g3/ble_v2.h"

#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "absl/synchronization/mutex.h"
#include "internal/platform/cancellation_flag_listener.h"
#include "internal/platform/implementation/ble_v2.h"
#include "internal/platform/implementation/shared/count_down_latch.h"
#include "internal/platform/logging.h"
#include "internal/platform/medium_environment.h"

namespace location {
namespace nearby {
namespace g3 {

namespace {

using ::location::nearby::api::ble_v2::BleAdvertisementData;
using ::location::nearby::api::ble_v2::PowerMode;

std::string PowerModeToName(PowerMode power_mode) {
  switch (power_mode) {
    case PowerMode::kUltraLow:
      return "UltraLow";
    case PowerMode::kLow:
      return "Low";
    case PowerMode::kMedium:
      return "Medium";
    case PowerMode::kHigh:
      return "High";
    case PowerMode::kUnknown:
      return "Unknown";
  }
}

}  // namespace

BleV2Socket::~BleV2Socket() {
  absl::MutexLock lock(&mutex_);
  DoClose();
}

void BleV2Socket::Connect(BleV2Socket& other) {
  absl::MutexLock lock(&mutex_);
  remote_socket_ = &other;
  input_ = other.output_;
}

InputStream& BleV2Socket::GetInputStream() {
  auto* remote_socket = GetRemoteSocket();
  CHECK(remote_socket != nullptr);
  return remote_socket->GetLocalInputStream();
}

OutputStream& BleV2Socket::GetOutputStream() { return GetLocalOutputStream(); }

BleV2Socket* BleV2Socket::GetRemoteSocket() {
  absl::MutexLock lock(&mutex_);
  return remote_socket_;
}

bool BleV2Socket::IsConnected() const {
  absl::MutexLock lock(&mutex_);
  return IsConnectedLocked();
}

bool BleV2Socket::IsClosed() const {
  absl::MutexLock lock(&mutex_);
  return closed_;
}

Exception BleV2Socket::Close() {
  absl::MutexLock lock(&mutex_);
  DoClose();
  return {Exception::kSuccess};
}

void BleV2Socket::DoClose() {
  if (!closed_) {
    remote_socket_ = nullptr;
    output_->GetOutputStream().Close();
    output_->GetInputStream().Close();
    input_->GetOutputStream().Close();
    input_->GetInputStream().Close();
    closed_ = true;
  }
}

bool BleV2Socket::IsConnectedLocked() const { return input_ != nullptr; }

InputStream& BleV2Socket::GetLocalInputStream() {
  absl::MutexLock lock(&mutex_);
  return output_->GetInputStream();
}

OutputStream& BleV2Socket::GetLocalOutputStream() {
  absl::MutexLock lock(&mutex_);
  return output_->GetOutputStream();
}

std::unique_ptr<api::ble_v2::BleSocket> BleV2ServerSocket::Accept() {
  absl::MutexLock lock(&mutex_);
  while (!closed_ && pending_sockets_.empty()) {
    cond_.Wait(&mutex_);
  }
  // whether or not we were running in the wait loop, return early if closed.
  if (closed_) return {};
  auto* remote_socket =
      pending_sockets_.extract(pending_sockets_.begin()).value();
  CHECK(remote_socket);

  auto local_socket = std::make_unique<BleV2Socket>();
  local_socket->Connect(*remote_socket);
  remote_socket->Connect(*local_socket);
  cond_.SignalAll();
  return local_socket;
}

bool BleV2ServerSocket::Connect(BleV2Socket& socket) {
  absl::MutexLock lock(&mutex_);
  if (closed_) return false;
  if (socket.IsConnected()) {
    NEARBY_LOGS(ERROR)
        << "Failed to connect to Ble server socket: already connected";
    return true;  // already connected.
  }
  // add client socket to the pending list
  pending_sockets_.insert(&socket);
  cond_.SignalAll();
  while (!socket.IsConnected()) {
    cond_.Wait(&mutex_);
    if (closed_) return false;
  }
  return true;
}

void BleV2ServerSocket::SetCloseNotifier(std::function<void()> notifier) {
  absl::MutexLock lock(&mutex_);
  close_notifier_ = std::move(notifier);
}

BleV2ServerSocket::~BleV2ServerSocket() {
  absl::MutexLock lock(&mutex_);
  DoClose();
}

Exception BleV2ServerSocket::Close() {
  absl::MutexLock lock(&mutex_);
  return DoClose();
}

Exception BleV2ServerSocket::DoClose() {
  bool should_notify = !closed_;
  closed_ = true;
  if (should_notify) {
    cond_.SignalAll();
    if (close_notifier_) {
      auto notifier = std::move(close_notifier_);
      mutex_.Unlock();
      // Notifier may contain calls to public API, and may cause deadlock, if
      // mutex_ is held during the call.
      notifier();
      mutex_.Lock();
    }
  }
  return {Exception::kSuccess};
}

BleV2Medium::BleV2Medium(api::BluetoothAdapter& adapter)
    : adapter_(static_cast<BluetoothAdapter*>(&adapter)) {
  adapter_->SetBleV2Medium(this);
  MediumEnvironment::Instance().RegisterBleV2Medium(*this);
}

BleV2Medium::~BleV2Medium() {
  adapter_->SetBleV2Medium(nullptr);
  MediumEnvironment::Instance().UnregisterBleV2Medium(*this);
}

bool BleV2Medium::StartAdvertising(
    const BleAdvertisementData& advertising_data,
    const BleAdvertisementData& scan_response_data, PowerMode power_mode) {
  NEARBY_LOGS(INFO)
      << "G3 Ble StartAdvertising:, advertising_data.service_uuids size="
      << advertising_data.service_uuids.size()
      << ", scan_response_data.service_data size="
      << scan_response_data.service_data.size()
      << ", power_mode=" << PowerModeToName(power_mode);

  absl::MutexLock lock(&mutex_);

  // Reassemble advertisement data from advertising and scan response
  // data.
  api::ble_v2::BleAdvertisementData advertisement_data;
  if (!advertising_data.service_uuids.empty()) {
    advertisement_data.service_uuids = advertising_data.service_uuids;
  } else {
    advertisement_data.service_uuids = scan_response_data.service_uuids;
  }
  advertisement_data.service_data = scan_response_data.service_data;

  MediumEnvironment::Instance().UpdateBleV2MediumForAdvertising(
      /*enabled=*/true, *this, adapter_->GetPeripheralV2(), advertisement_data);
  return true;
}

bool BleV2Medium::StopAdvertising() {
  NEARBY_LOGS(INFO) << "G3 Ble StopAdvertising";
  absl::MutexLock lock(&mutex_);

  BleAdvertisementData empty_advertisement_data = {};
  MediumEnvironment::Instance().UpdateBleV2MediumForAdvertising(
      /*enabled=*/false, *this, /*mutable=*/adapter_->GetPeripheralV2(),
      empty_advertisement_data);
  return true;
}

bool BleV2Medium::StartScanning(const std::vector<std::string>& service_uuids,
                                PowerMode power_mode, ScanCallback callback) {
  NEARBY_LOGS(INFO) << "G3 Ble StartScanning";
  absl::MutexLock lock(&mutex_);

  MediumEnvironment::Instance().UpdateBleV2MediumForScanning(
      /*enabled=*/true, service_uuids.front(), std::move(callback), *this);
  return true;
}

bool BleV2Medium::StopScanning() {
  NEARBY_LOGS(INFO) << "G3 Ble StopScanning";
  absl::MutexLock lock(&mutex_);

  MediumEnvironment::Instance().UpdateBleV2MediumForScanning(
      /*enabled=*/false,
      /*service_uuid=*/{}, /*callback=*/{}, *this);
  return true;
}

std::unique_ptr<api::ble_v2::GattServer> BleV2Medium::StartGattServer() {
  return std::make_unique<GattServer>();
}

std::unique_ptr<api::ble_v2::GattClient> BleV2Medium::ConnectToGattServer(
    api::ble_v2::BlePeripheral& peripheral, PowerMode power_mode) {
  return std::make_unique<GattClient>();
}

std::optional<api::ble_v2::GattCharacteristic>
BleV2Medium::GattServer::CreateCharacteristic(
    absl::string_view service_uuid, absl::string_view characteristic_uuid,
    const std::vector<api::ble_v2::GattCharacteristic::Permission>& permissions,
    const std::vector<api::ble_v2::GattCharacteristic::Property>& properties) {
  api::ble_v2::GattCharacteristic characteristic = {
      .uuid = std::string(characteristic_uuid),
      .servie_uuid = std::string(service_uuid)};
  return characteristic;
}

bool BleV2Medium::GattServer::UpdateCharacteristic(
    const api::ble_v2::GattCharacteristic& characteristic,
    const location::nearby::ByteArray& value) {
  NEARBY_LOGS(INFO)
      << "G3 Ble GattServer UpdateCharacteristic, characteristic=("
      << characteristic.servie_uuid << "," << characteristic.uuid
      << "), value = " << absl::BytesToHexString(value.data());
  MediumEnvironment::Instance().InsertBleV2MediumGattCharacteristics(
      characteristic, value);
  return true;
}

void BleV2Medium::GattServer::Stop() {
  NEARBY_LOGS(INFO) << "G3 Ble GattServer Stop";
  MediumEnvironment::Instance().ClearBleV2MediumGattCharacteristics();
}

bool BleV2Medium::GattClient::DiscoverService(const std::string& service_uuid) {
  absl::MutexLock lock(&mutex_);
  NEARBY_LOGS(INFO) << "G3 Ble GattClient DiscoverService, service_uuid"
                    << service_uuid;
  if (!is_gatt_connection_) {
    return false;
  }

  // Search if service is existed.
  return MediumEnvironment::Instance().ContainsBleV2MediumGattCharacteristics(
      service_uuid, "");
}

std::optional<api::ble_v2::GattCharacteristic>
BleV2Medium::GattClient::GetCharacteristic(
    absl::string_view service_uuid, absl::string_view characteristic_uuid) {
  absl::MutexLock lock(&mutex_);
  NEARBY_LOGS(INFO) << "G3 Ble GattClient GetCharacteristic, service_uuid="
                    << service_uuid
                    << ", characteristic_uuid=" << characteristic_uuid;
  if (!is_gatt_connection_) {
    return {};
  }

  // Search gatt_characteristic by uuid and if found return the
  // gatt_characteristic.
  api::ble_v2::GattCharacteristic characteristic = {};
  if (MediumEnvironment::Instance().ContainsBleV2MediumGattCharacteristics(
          service_uuid, characteristic_uuid)) {
    characteristic = {.uuid = std::string(characteristic_uuid),
                      .servie_uuid = std::string(service_uuid)};
  }
  NEARBY_LOGS(INFO)
      << "G3 Ble GattClient GetCharacteristic, found characteristic=("
      << characteristic.servie_uuid << "," << characteristic.uuid << ")";

  return characteristic;
}

std::optional<ByteArray> BleV2Medium::GattClient::ReadCharacteristic(
    const api::ble_v2::GattCharacteristic& characteristic) {
  absl::MutexLock lock(&mutex_);
  if (!is_gatt_connection_) {
    return {};
  }

  ByteArray value =
      MediumEnvironment::Instance().ReadBleV2MediumGattCharacteristics(
          characteristic);
  NEARBY_LOGS(INFO) << "G3 Ble ReadCharacteristic, characteristic=("
                    << characteristic.servie_uuid << "," << characteristic.uuid
                    << "), value = " << absl::BytesToHexString(value.data());
  return std::move(value);
}

void BleV2Medium::GattClient::Disconnect() {
  absl::MutexLock lock(&mutex_);
  NEARBY_LOGS(INFO) << "G3 Ble GattClient Disconnect";
  is_gatt_connection_ = false;
}

std::unique_ptr<api::ble_v2::BleServerSocket> BleV2Medium::OpenServerSocket(
    const std::string& service_id) {
  auto server_socket = std::make_unique<BleV2ServerSocket>();
  server_socket->SetCloseNotifier([this, service_id]() {
    absl::MutexLock lock(&mutex_);
    server_sockets_.erase(service_id);
  });
  NEARBY_LOGS(INFO) << "G3 Ble Adding server socket: medium=" << this
                    << ", service_id=" << service_id;
  absl::MutexLock lock(&mutex_);
  server_sockets_.insert({service_id, server_socket.get()});
  return server_socket;
}

std::unique_ptr<api::ble_v2::BleSocket> BleV2Medium::Connect(
    const std::string& service_id, api::ble_v2::PowerMode power_mode,
    api::ble_v2::BlePeripheral& remote_peripheral,
    CancellationFlag* cancellation_flag) {
  std::string socket_name = "XXX";
  NEARBY_LOGS(INFO) << "G3 Ble Connect [self]: medium=" << this
                    << ", adapter=" << &GetAdapter()
                    << ", peripheral=" << &GetAdapter().GetPeripheralV2()
                    << ", service_id=" << service_id;
  // First, find an instance of remote medium, that exposed this peripheral.
  auto& remote_adapter =
      static_cast<BleV2Peripheral&>(remote_peripheral).GetAdapter();
  auto* remote_medium =
      static_cast<BleV2Medium*>(remote_adapter.GetBleV2Medium());
  if (!remote_medium) {
    return {};
  }

  BleV2ServerSocket* remote_server_socket = nullptr;
  NEARBY_LOGS(INFO) << "G3 Ble Connect [peer]: medium=" << remote_medium
                    << ", adapter=" << &remote_adapter
                    << ", peripheral=" << &remote_peripheral
                    << ", service_id=" << service_id;
  // Then, find our server socket context in this medium.
  {
    absl::MutexLock medium_lock(&remote_medium->mutex_);
    auto item = remote_medium->server_sockets_.find(service_id);
    remote_server_socket =
        item != server_sockets_.end() ? item->second : nullptr;
    if (remote_server_socket == nullptr) {
      NEARBY_LOGS(ERROR)
          << "G3 Ble Failed to find Ble Server socket: service_id="
          << service_id;
      return {};
    }
  }

  if (cancellation_flag->Cancelled()) {
    NEARBY_LOGS(ERROR) << "G3 BLE Connect: Has been cancelled: "
                          "service_id="
                       << service_id;
    return {};
  }

  CancellationFlagListener listener(
      cancellation_flag, [&remote_server_socket]() {
        NEARBY_LOGS(INFO) << "G3 Ble Cancel Connect.";
        if (remote_server_socket != nullptr) {
          remote_server_socket->Close();
        }
      });

  auto socket = std::make_unique<BleV2Socket>();
  // Finally, Request to connect to this socket.
  if (!remote_server_socket->Connect(*socket)) {
    NEARBY_LOGS(ERROR) << "G3 Ble Failed to connect to existing Ble "
                          "Server socket: service_id="
                       << service_id;
    return {};
  }
  NEARBY_LOGS(INFO) << "G3 Ble Connect to socket=" << socket.get();
  return socket;
}

}  // namespace g3
}  // namespace nearby
}  // namespace location
