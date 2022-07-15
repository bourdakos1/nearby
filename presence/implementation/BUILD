# Copyright 2020 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
licenses(["notice"])

cc_library(
    name = "internal",
    srcs = [],
    hdrs = [
        "broadcast_manager.h",
        "credential_manager.h",
        "mock_service_controller.h",
        "scan_manager.h",
        "service_controller.h",
        "service_controller_impl.h",
    ],
    visibility = [
        "//third_party/nearby/presence:__subpackages__",
    ],
    deps = [
        "//internal/platform:base",
        "//internal/platform:comm",
        "//third_party/nearby/presence/implementation/mediums",
    ],
)