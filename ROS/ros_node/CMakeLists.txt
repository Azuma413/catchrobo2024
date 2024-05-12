cmake_minimum_required(VERSION 3.8)
project(mpu9250)

find_package(ament_cmake_auto REQUIRED)

#xmlから依存を見つける
ament_auto_find_build_dependencies()

#実行ファイルの生成
ament_auto_add_executable(
quat_node src/quat_node.cpp
)

#構文チェック
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#ライブラリのビルド
ament_auto_package()