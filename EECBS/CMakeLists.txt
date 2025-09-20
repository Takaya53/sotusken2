# =========================
# CMakeLists.txt (完全版)
# =========================
cmake_minimum_required(VERSION 3.20)
project(eecbs CXX)

# ---- C++標準 ----
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- ビルドタイプ既定 ----
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

# ---- Apple Silicon/Homebrew の探索パス ----
if(APPLE AND EXISTS /opt/homebrew)
  list(PREPEND CMAKE_PREFIX_PATH /opt/homebrew)
endif()

# ---- CMP0167: FindBoost を使う前提なので OLD にして警告回避 ----
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 OLD)
endif()

# ---- Boost 設定：Config パッケージは使わず FindBoost を強制 ----
set(Boost_NO_BOOST_CMAKE ON)
# （お好みで）静的/マルチスレッド指定
set(Boost_USE_STATIC_LIBS        OFF)
set(Boost_USE_MULTITHREADED       ON)
set(Boost_USE_STATIC_RUNTIME     OFF)

# ---- Boost の“必要コンポーネント”は program_options のみ
#      Boost.System はヘッダオンリーで使う（下でマクロ定義）
find_package(Boost 1.70 REQUIRED COMPONENTS program_options)

# ---- ヘッダ/ソース ----
include_directories("inc")
file(GLOB SOURCES "src/*.cpp")

# ---- 実行ファイル ----
add_executable(eecbs ${SOURCES})

# ---- Boost.System をヘッダオンリー運用 ----
# これで libboost_system を探してリンクする必要がなくなる
target_compile_definitions(eecbs PRIVATE
  BOOST_ERROR_CODE_HEADER_ONLY
  BOOST_SYSTEM_NO_DEPRECATED
)

# ---- Boost.Program_options をリンク（インポートターゲット優先、無ければ変数でフォールバック）----
if(TARGET Boost::program_options)
  target_link_libraries(eecbs PRIVATE Boost::program_options)
else()
  target_include_directories(eecbs PRIVATE ${Boost_INCLUDE_DIRS})
  target_link_libraries(eecbs PRIVATE ${Boost_LIBRARIES})
endif()

# ---- 任意の警告/最適化（必要なら）
# target_compile_options(eecbs PRIVATE -Wall -Wextra -Wpedantic)