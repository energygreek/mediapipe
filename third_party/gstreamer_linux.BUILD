# Copyright 2019 The MediaPipe Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

licenses(["notice"])  # LGPL

exports_files(["LICENSE"])

cc_library(
    name = "libgstreamer",
    hdrs = glob([
	    "lib64/glib-2.0/include/glibconfig.h",
        "lib/x86_64-linux-gnu/glib-2.0/include/glibconfig.h",
        "include/glib-2.0/**/*",
        "include/gstreamer-1.0/**/*.h*",
    ]),
    includes = [
	    "lib64/glib-2.0/include",
        "lib/x86_64-linux-gnu/glib-2.0/include",
        "include/gstreamer-1.0/",
        "include/glib-2.0/",
    ],
    linkopts = [
        "-l:libgstreamer-1.0.so",
        "-l:libgobject-2.0.so",
        "-l:libglib-2.0.so",
    ],
    visibility = ["//visibility:public"],
)
