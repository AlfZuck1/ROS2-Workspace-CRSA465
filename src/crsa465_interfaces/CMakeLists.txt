cmake_minimum_required(VERSION 3.8)
project(crsa465_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencias
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generación de interfaces (srv)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Command.srv"
)

# Exportar dependencias de rosidl
ament_export_dependencies(rosidl_default_runtime)

# Pruebas automáticas
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Finalizar paquete
ament_package()
