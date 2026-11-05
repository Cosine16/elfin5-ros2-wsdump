"""Exercise vendor preparation and build routing without ROS or robot hardware."""

import ctypes
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

import pytest


REPO = Path(__file__).resolve().parents[1]
VENDOR = REPO / "third_party/elfin_robot_ros2"


def run(*args, **kwargs):
    result = subprocess.run(args, text=True, capture_output=True, **kwargs)
    assert result.returncode == 0, result.stdout + result.stderr
    return result.stdout


@pytest.fixture
def checkout(tmp_path):
    root = tmp_path / "relocated workstation"
    vendor = root / "third_party/elfin_robot_ros2"
    for package in ("elfin_ethercat_driver", "elfin_ros_control"):
        path = vendor / package / "CMakeLists.txt"
        path.parent.mkdir(parents=True)
        path.write_text(run("git", "-C", str(VENDOR), "show", f"HEAD:{package}/CMakeLists.txt"))
    run("git", "init", "-q", str(vendor))
    run("git", "-C", str(vendor), "add", ".")
    run("git", "-C", str(vendor), "-c", "user.name=Integration test",
        "-c", "user.email=test@example.invalid", "commit", "-qm", "fixture baseline")
    revision = run("git", "-C", str(vendor), "rev-parse", "HEAD")
    (root / "third_party/elfin_robot_ros2.revision").write_text(revision)
    shutil.copytree(REPO / "third_party/patches", root / "third_party/patches")
    (root / "scripts").mkdir()
    for name in ("prepare_vendor.sh", "build_vendor.sh", "build_apps.sh", "build_handeye.sh",
                 "elfin_env.sh", "elfin_env.zsh", "start_real.py"):
        shutil.copy2(REPO / "scripts" / name, root / "scripts" / name)
    return root, vendor


def test_preparation_is_repeatable_and_rejects_wrong_revision(checkout):
    root, vendor = checkout
    script = str(root / "scripts/prepare_vendor.sh")
    run("bash", script, "--check", cwd="/")
    assert not run("git", "-C", str(vendor), "diff")
    run("bash", script, cwd="/")
    patched = run("git", "-C", str(vendor), "diff")
    assert patched
    run("bash", script, cwd="/")
    assert run("git", "-C", str(vendor), "diff") == patched
    (root / "third_party/elfin_robot_ros2.revision").write_text("0" * 40 + "\n")
    rejected = subprocess.run(["bash", script], text=True, capture_output=True)
    assert rejected.returncode != 0 and "revision mismatch" in rejected.stderr
    assert run("git", "-C", str(vendor), "diff") == patched


def test_preparation_preserves_conflicting_local_changes(checkout):
    root, vendor = checkout
    path = vendor / "elfin_ros_control/CMakeLists.txt"
    path.write_text(path.read_text().replace("find_package(elfin_ethercat_driver)",
                                            "find_package(elfin_ethercat_driver QUIET)"))
    before = run("git", "-C", str(vendor), "diff")
    result = subprocess.run(["bash", str(root / "scripts/prepare_vendor.sh")],
                            text=True, capture_output=True)
    assert result.returncode != 0 and "Patch conflicts" in result.stderr
    assert run("git", "-C", str(vendor), "diff") == before


def test_build_profiles_use_separate_sources_and_outputs(checkout):
    root, _ = checkout
    bindir = root / "bin"
    bindir.mkdir()
    recorder = bindir / "colcon"
    recorder.write_text(f"#!{sys.executable}\n" +
        "import json, os, sys\nfrom pathlib import Path\n" +
        "Path(os.environ['TRACE']).write_text(json.dumps({'args':sys.argv[1:], " +
        "'underlay':os.environ.get('VENDOR_LOADED'), 'stale':os.environ.get('CMAKE_PREFIX_PATH')}))\n")
    recorder.chmod(0o755)
    ros2 = bindir / "ros2"
    ros2.write_text("#!/bin/sh\nexit 0\n")
    ros2.chmod(0o755)
    ros_setup = root / "ros-setup.bash"
    ros_setup.write_text("export SYSTEM_ROS_LOADED=1\n")
    vendor_output = root / "custom vendor output"
    setup = vendor_output / "install/setup.bash"
    setup.parent.mkdir(parents=True)
    setup.write_text("export VENDOR_LOADED=1\n")
    trace = root / "trace.json"
    env = {k: v for k, v in os.environ.items()
           if k not in ("APP_BUILD_DIR", "HANDEYE_WS", "VENDOR_LOADED")}
    env.update(PATH=f"{bindir}:{env['PATH']}", ROS_SETUP=str(ros_setup), TRACE=str(trace),
               VENDOR_BUILD_DIR=str(vendor_output), CMAKE_PREFIX_PATH="/old/workspace")
    for profile, source, output, underlay in (
        ("vendor", root / "third_party/elfin_robot_ros2", vendor_output, None),
        ("apps", root / "apps", root / ".build/apps", "1"),
        ("handeye", root / "third_party/elfin_robot_ros2", root / ".build/handeye", None),
    ):
        run("bash", str(root / f"scripts/build_{profile}.sh"), "--packages-select", "probe", cwd="/", env=env)
        data = json.loads(trace.read_text())
        assert str(source) in data["args"]
        assert str(output / "install") in data["args"]
        assert data["args"][-2:] == ["--packages-select", "probe"]
        assert data["underlay"] == underlay and data["stale"] is None


def test_runtime_entries_separate_vendor_source_and_custom_build_directory(checkout):
    root, vendor = checkout
    output = root / "custom vendor output"
    application = root / ".build/apps/install"
    application.mkdir(parents=True)
    (output / "install").mkdir(parents=True)
    ros = root / "ros-setup"
    ros.write_text("echo ros-setup-chatter\n")
    for shell in ("bash", "zsh"):
        (output / f"install/setup.{shell}").write_text("echo vendor-setup-chatter\nexport VENDOR_LOADED=1\n")
        (application / f"setup.{shell}").write_text("export APPLICATION_LOADED=1\n")
    env = {**os.environ, "VENDOR_BUILD_DIR": str(output), "ROS_SETUP": str(ros), "ROS_ZSH_SETUP": str(ros)}
    env.pop("APP_BUILD_DIR", None)
    for shell, suffix in (("bash", "sh"), ("zsh", "zsh")):
        if shutil.which(shell):
            result = run(shell, "-c", 'source "$1" && test "$VENDOR_LOADED:$APPLICATION_LOADED" = 1:1',
                         "test", str(root / f"scripts/elfin_env.{suffix}"), cwd="/", env=env)
            assert "vendor-setup-chatter" in result
    mcp = root / "apps/bridges/mcp"
    (mcp / ".venv/bin").mkdir(parents=True)
    shutil.copy2(REPO / "apps/bridges/mcp/run_server.sh", mcp / "run_server.sh")
    interpreter = mcp / ".venv/bin/python"
    interpreter.write_text('#!/bin/sh\nprintf "%s\\n" "$VENDOR_LOADED"\n')
    interpreter.chmod(0o755)
    assert run("bash", str(mcp / "run_server.sh"), cwd="/", env=env) == "1\n"
    config = run(sys.executable, "-c",
        "import json, runpy, sys; m=runpy.run_path(sys.argv[1]); "
        "print(json.dumps([m['DRIVER_WS'], m['BRINGUP_CONFIG']]))",
        str(root / "scripts/start_real.py"), env=env)
    assert json.loads(config) == [str(output), str(vendor / "elfin_robot_bringup/config")]


@pytest.mark.skipif(not shutil.which("cmake") or not shutil.which("ar") or not shutil.which("c++"),
                    reason="CMake and C++ build tools required")
def test_driver_installs_and_links_outside_source_workspace(checkout):
    """Use placeholder ROS dependencies and C++ bodies to test the actual CMake files.

    This validates installation/linkage, not ROS API or hardware compatibility.
    """
    root, vendor = checkout
    run("bash", str(root / "scripts/prepare_vendor.sh"))
    deps = root / "placeholder dependencies"
    ament = deps / "share/ament_cmake/cmake/ament_cmakeConfig.cmake"
    ament.parent.mkdir(parents=True)
    ament.write_text(r'''
macro(ament_target_dependencies target)
  foreach(dep ${ARGN})
    if(${dep}_INCLUDE_DIRS)
      target_include_directories(${target} PRIVATE ${${dep}_INCLUDE_DIRS})
    endif()
    if(${dep}_LIBRARIES)
      target_link_libraries(${target} ${${dep}_LIBRARIES})
    endif()
  endforeach()
endmacro()
macro(ament_export_include_directories)
  set(probe_export_includes TRUE)
endmacro()
macro(ament_export_libraries)
  set(probe_export_libraries ${ARGN})
endmacro()
macro(ament_export_dependencies)
endmacro()
macro(pluginlib_export_plugin_description_file)
endmacro()
macro(ament_package)
  set(config "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake")
  file(WRITE "${config}" "get_filename_component(prefix \"\${CMAKE_CURRENT_LIST_DIR}/../../..\" ABSOLUTE)\n")
  if(probe_export_includes)
    file(APPEND "${config}" "set(${PROJECT_NAME}_INCLUDE_DIRS \"\${prefix}/include\")\n")
  endif()
  foreach(lib ${probe_export_libraries})
    file(APPEND "${config}" "find_library(${PROJECT_NAME}_LIBRARIES NAMES ${lib} PATHS \"\${prefix}/lib\" NO_DEFAULT_PATH REQUIRED)\n")
  endforeach()
  install(FILES "${config}" DESTINATION "share/${PROJECT_NAME}/cmake")
endmacro()
''')
    contents = "\n".join(p.read_text() for p in vendor.glob("*/CMakeLists.txt"))
    for name in set(re.findall(r"find_package\((\w+)", contents)) - {"ament_cmake", "elfin_ethercat_driver"}:
        config = deps / f"share/{name}/cmake/{name}Config.cmake"
        config.parent.mkdir(parents=True)
        config.write_text(f"set({name}_FOUND TRUE)\n")
    (deps / "lib").mkdir()
    for name in ("boost_system", "boost_thread"):
        run("ar", "rcs", str(deps / f"lib/lib{name}.a"))
    driver = vendor / "elfin_ethercat_driver"
    control = vendor / "elfin_ros_control"
    header = driver / "include/elfin_ethercat_driver/elfin_ethercat_driver.h"
    header.parent.mkdir(parents=True)
    header.write_text('extern "C" int vendor_probe();\n')
    for name in ("elfin_ethercat_client", "elfin_ethercat_io_client", "elfin_ethercat_manager"):
        (header.parent / f"{name}.h").write_text("// placeholder\n")
    for directory in (driver / "src", control / "src", control / "include"):
        directory.mkdir(parents=True)
    for name in ("elfin_ethercat_driver", "elfin_ethercat_client", "elfin_ethercat_io_client", "elfin_ethercat_manager"):
        (driver / f"src/{name}.cpp").write_text('extern "C" int vendor_probe() { return 42; }\n'
                                               if name == "elfin_ethercat_driver" else "// placeholder\n")
    (driver / "src/elfin_ethercat_driver_node.cpp").write_text(
        '#include "elfin_ethercat_driver/elfin_ethercat_driver.h"\nint main() { return vendor_probe() == 42 ? 0 : 1; }\n')
    (control / "src/elfin_hardware_interface.cpp").write_text(
        '#include "elfin_ethercat_driver/elfin_ethercat_driver.h"\nextern "C" int interface_probe() { return vendor_probe(); }\n')
    (control / "src/elfin_hardware_onlyposition_interface.cpp").write_text("// placeholder\n")
    driver_prefix = root / "install driver"
    control_prefix = root / "install control"
    for source, prefix in ((driver, driver_prefix), (control, control_prefix)):
        build = root / "build outside source" / source.name
        run("cmake", "-S", str(source), "-B", str(build), "-DBUILD_TESTING=OFF",
            f"-DCMAKE_PREFIX_PATH={driver_prefix};{deps}", f"-DCMAKE_INSTALL_PREFIX={prefix}",
            f'-DCMAKE_SHARED_LINKER_FLAGS=-L"{deps / "lib"}"',
            f'-DCMAKE_EXE_LINKER_FLAGS=-L"{deps / "lib"}"')
        run("cmake", "--build", str(build), "--target", "install", "-j2")
    run(str(driver_prefix / "lib/elfin_ethercat_driver/elfin_ethercat_driver"))
    interface = ctypes.CDLL(str(control_prefix / "lib/elfin_ros_control/libelfin_hardware_interface.so"))
    assert interface.interface_probe() == 42
