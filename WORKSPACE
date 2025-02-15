workspace(name = "ponyai_public_course")

load("//utils/bazel:pony_http_archive.bzl", "pony_http_archive")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# gtest
pony_http_archive(
    name = "googletest",
    build_file = "utils/bazel/googletest.BUILD",
    sha256 = "58a6f4277ca2bc8565222b3bbd58a177609e9c488e8a72649359ba51450db7d8",
    strip_prefix = "googletest-release-1.8.0",
    urls = ["https://github.com/google/googletest/archive/release-1.8.0.tar.gz"],
)

# glog
pony_http_archive(
    name = "glog",
    build_file = "utils/bazel/glog.BUILD",
    sha256 = "7580e408a2c0b5a89ca214739978ce6ff480b5e7d8d7698a2aa92fadc484d1e0",
    strip_prefix = "glog-0.3.5",
    urls = ["https://github.com/google/glog/archive/v0.3.5.tar.gz"],
)

# gflags
pony_http_archive(
    name = "gflags",
    sha256 = "ae27cdbcd6a2f935baa78e4f21f675649271634c092b1be01469440495609d0e",
    strip_prefix = "gflags-2.2.1",
    urls = ["https://github.com/gflags/gflags/archive/v2.2.1.tar.gz"],
)

# eigen
pony_http_archive(
    name = "eigen",
    build_file = "utils/bazel/eigen.BUILD",
    sha256 = "4286e8f1fabbd74f7ec6ef8ef31c9dbc6102b9248a8f8327b04f5b68da5b05e1",
    strip_prefix = "eigen-eigen-5a0156e40feb",
    urls = ["http://bitbucket.org/eigen/eigen/get/3.3.4.tar.gz"],
)

# boost
pony_http_archive(
    name = "boost",
    build_file = "utils/bazel/boost.BUILD",
    sha256 = "a004d9b3fa95e956383693b86fce1b68805a6f71c2e68944fa813de0fb8c8102",
    strip_prefix = "boost_1_58_0",
    urls = [
        "https://cloud.tsinghua.edu.cn/lib/3d27e7a9-7b80-4ac5-b0a0-6c90946f1b53/file/boost_1_58_0.tar.gz?dl=1",
    ],
)

# opencv
new_local_repository(
    name = "opencv",
    build_file = "utils/bazel/opencv.BUILD",
    path = "/usr",
)

# Qt5
new_local_repository(
    name = "qt",
    build_file = "utils/bazel/qt.BUILD",
    path = "/usr",
)

# protobuf
pony_http_archive(
    name = "com_google_protobuf",
    patch_file = "utils/bazel/protobuf.patch",
    sha256 = "8e0236242106e680b4f9f576cc44b8cd711e948b20a9fc07769b0a20ceab9cc4",
    strip_prefix = "protobuf-3.4.1",
    urls = ["https://cloud.tsinghua.edu.cn/lib/3d27e7a9-7b80-4ac5-b0a0-6c90946f1b53/file/protobuf-3.4.1.tar.gz?dl=1"],
)

# GLM
pony_http_archive(
    name = "glm",
    build_file = "utils/bazel/glm.BUILD",
    sha256 = "9f9f520ec7fb8c20c69d6b398ed928a2448c6a3245cbedb8631a56a987c38660",
    strip_prefix = "glm",
    urls = [
        "https://cloud.tsinghua.edu.cn/lib/3d27e7a9-7b80-4ac5-b0a0-6c90946f1b53/file/glm-0.9.8.5.zip?dl=1",
    ],
)

# opengl
new_local_repository(
    name = "opengl",
    build_file = "utils/bazel/opengl.BUILD",
    path = "/usr",
)

# png
pony_http_archive(
    name = "png",
    build_file = "utils/bazel/png.BUILD",
    sha256 = "d5bc743ac338bd454e330279f70534f5a31ea4c2cd3ee3ce76fd6e7f17fd3950",
    strip_prefix = "libpng-1.2.59",
    urls = [
        "https://github.com/glennrp/libpng/archive/v1.2.59.tar.gz",
    ],
)

# jpeg-turbo
pony_http_archive(
    name = "jpeg_turbo",
    build_file = "utils/bazel/jpeg-turbo.BUILD",
    sha256 = "1a17020f859cb12711175a67eab5c71fc1904e04b587046218e36106e07eabde",
    strip_prefix = "libjpeg-turbo-1.5.3",
    urls = [
        "https://github.com/libjpeg-turbo/libjpeg-turbo/archive/1.5.3.tar.gz",
    ],
)

# zlib
pony_http_archive(
    name = "zlib",
    build_file = "utils/bazel/zlib.BUILD",
    sha256 = "c3e5e9fdd5004dcb542feda5ee4f0ff0744628baf8ed2dd5d66f8ca1197cb1a1",
    strip_prefix = "zlib-1.2.11",
    urls = [
        "https://cloud.tsinghua.edu.cn/lib/3d27e7a9-7b80-4ac5-b0a0-6c90946f1b53/file/zlib-1.2.11.tar.gz?dl=1",
    ],
)

# fmt
pony_http_archive(
    name = "fmt",
    build_file = "utils/bazel/fmt.BUILD",
    sha256 = "46628a2f068d0e33c716be0ed9dcae4370242df135aed663a180b9fd8e36733d",
    strip_prefix = "fmt-4.1.0",
    urls = [
        "https://github.com/fmtlib/fmt/archive/4.1.0.tar.gz",
    ],
)

# nanoflann
pony_http_archive(
    name = "nanoflann",
    build_file = "utils/bazel/nanoflann.BUILD",
    patch_file = "utils/bazel/nanoflann.patch",
    sha256 = "5ef4dfb23872379fe9eb306aabd19c9df4cae852b72a923af01aea5e8d7a59c3",
    urls = [
        "https://github.com/jlblancoc/nanoflann/archive/v1.2.3.tar.gz",
    ],
    strip_prefix = "nanoflann-1.2.3",
)

pony_http_archive(
    name = "bazel_skylib",
    sha256 = "bbccf674aa441c266df9894182d80de104cabd19be98be002f6d478aaa31574d",
    strip_prefix = "bazel-skylib-2169ae1c374aab4a09aa90e65efe1a3aad4e279b",
    urls = [
        "https://github.com/bazelbuild/bazel-skylib/archive/2169ae1c374aab4a09aa90e65efe1a3aad4e279b.tar.gz",
    ],
)
