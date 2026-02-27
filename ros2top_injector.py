#!/usr/bin/env python3
"""
ros2top injector - Automatically injects ros2top registration, heartbeat,
and shutdown hooks into ROS2 C++ and Python nodes.

Handles:
  - package.xml: adds ros2top + nlohmann_json dependencies
  - CMakeLists.txt: root vs subdirectory aware injection
  - C++ .cpp nodes: register_node, heartbeat timer, out-of-class method impls, destructor
  - C++ .hpp headers: member variable + method declarations in class body
  - Python nodes: register_node, heartbeat timer, heartbeat callback, shutdown method
"""
import os
import re
import sys


# ──────────────────────────────────────────────────────────────────────
#  package.xml
# ──────────────────────────────────────────────────────────────────────
def process_package_xml(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    if '<depend>ros2top</depend>' in content:
        return False

    depend_insertion = "  <depend>ros2top</depend>\n  <depend>nlohmann_json</depend>\n"

    if '</depend>' in content:
        content = re.sub(
            r'(</depend>)\s*(?=<export>|</package>)',
            r'\1\n' + depend_insertion, content, count=1)
    else:
        content = re.sub(r'(</package>)', depend_insertion + r'\1', content, count=1)

    with open(filepath, 'w') as f:
        f.write(content)
    return True


# ──────────────────────────────────────────────────────────────────────
#  CMakeLists.txt  (root vs subdirectory aware)
# ──────────────────────────────────────────────────────────────────────
def _is_root_cmake(content):
    """A root (package-level) CMakeLists contains project() and usually ament_package()."""
    return bool(re.search(r'^\s*project\s*\(', content, re.MULTILINE))


def process_cmakelists(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    modified = False
    is_root = _is_root_cmake(content)

    if is_root:
        # ── Root CMakeLists ──────────────────────────────────────────

        # 1. Add CMAKE_PREFIX_PATH hint (before first find_package if possible)
        if 'CMAKE_PREFIX_PATH' not in content and '$ENV{HOME}/.local' not in content:
            m = re.search(r'^(\s*find_package\s*\()', content, re.MULTILINE)
            if m:
                prefix_line = '# ros2top is installed via pip; hint the pip prefix.\nlist(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/.local")\n\n'
                content = content[:m.start()] + prefix_line + content[m.start():]
                modified = True

        # 2. Add find_package(ros2top REQUIRED) + find_package(nlohmann_json REQUIRED)
        #    Insert after the FIRST find_package to avoid landing inside
        #    conditional blocks like if(BUILD_TESTING).
        if 'find_package(ros2top REQUIRED)' not in content:
            find_pkg_insertion = "find_package(ros2top REQUIRED)\nfind_package(nlohmann_json REQUIRED)\n"
            first_match = re.search(r'find_package\([^\)]+\)\n', content)
            if first_match:
                insert_pos = first_match.end()
                content = content[:insert_pos] + find_pkg_insertion + content[insert_pos:]
                modified = True

        # 3. Add to set(dependencies ...) block
        if 'set(dependencies' in content:
            m = re.search(r'(set\(\s*dependencies[^)]*)\)', content)
            if m and 'ros2top' not in m.group(0):
                content = re.sub(
                    r'(set\(\s*dependencies[^)]*)\)',
                    r'\1\n  ros2top\n  nlohmann_json\n)', content)
                modified = True
        elif 'ament_target_dependencies' in content:
            def _add_ros2top_dep(m):
                if 'ros2top' in m.group(0):
                    return m.group(0)
                return m.group(1) + ' ros2top nlohmann_json)'

            new_content = re.sub(
                r'(ament_target_dependencies\([^)]+)\)', _add_ros2top_dep, content)
            if new_content != content:
                content = new_content
                modified = True

    else:
        # ── Subdirectory CMakeLists ──────────────────────────────────
        # Add ros2top::ros2top to all target_link_libraries.
        # This is necessary even for pure library targets because process_hpp
        # injects `#include <ros2top/ros2top.hpp>` into header files, which means
        # any source file including those headers needs the ros2top include paths.
        if 'target_link_libraries(' in content and 'ros2top::ros2top' not in content:
            def _add_ros2top_link(m):
                return m.group(1) + ' ros2top::ros2top)'

            new_content = re.sub(
                r'(target_link_libraries\([^)]+)\)', _add_ros2top_link, content)
            if new_content != content:
                content = new_content
                modified = True

        if 'ament_target_dependencies' in content and 'ros2top' not in content:
            def _add_ros2top_dep(m):
                return m.group(1) + '\n ros2top nlohmann_json)'

            new_content = re.sub(
                r'(ament_target_dependencies\([^)]+)\)', _add_ros2top_dep, content)
            if new_content != content:
                content = new_content
                modified = True

    if modified:
        with open(filepath, 'w') as f:
            f.write(content)
        return True
    return False


# ──────────────────────────────────────────────────────────────────────
#  Helpers for C++ processing
# ──────────────────────────────────────────────────────────────────────
def _find_include_block_end(content):
    """Return char offset just after the last ``#include`` in the first
    contiguous block.  Blank lines and ``//`` comments between
    ``#include`` lines are tolerated; anything else ends the block."""
    block_end = 0
    in_block = False
    for m in re.finditer(r'^[ \t]*#include\s+[<"][^>"]+[>"]\s*\n', content, re.MULTILINE):
        if not in_block:
            block_end = m.end()
            in_block = True
        else:
            gap = content[block_end:m.start()]
            # Allow blank lines and single-line // comments inside the block
            if re.fullmatch(r'[\s]*(?://[^\n]*\n[\s]*)*', gap):
                block_end = m.end()
            else:
                break  # Non-trivial code separates this include from the block
    return block_end


def _find_body_brace(content, start):
    """Starting from *start*, find the first ``{`` that opens a function /
    constructor body.  Brace-initialised members (``name_{...}``) are
    correctly skipped by checking whether the ``{`` is immediately
    preceded (ignoring whitespace) by an identifier character."""
    pos = start
    while pos < len(content):
        if content[pos] == '{':
            pre = content[:pos].rstrip()
            if pre and (pre[-1].isalnum() or pre[-1] == '_'):
                # Brace-init – skip to matching }
                depth = 1
                pos += 1
                while pos < len(content) and depth > 0:
                    if content[pos] == '{':
                        depth += 1
                    elif content[pos] == '}':
                        depth -= 1
                    pos += 1
                continue
            return pos
        pos += 1
    return None


def _find_matching_brace(content, open_pos):
    """Return the position *after* the ``}`` that matches the ``{`` at
    *open_pos*."""
    depth = 1
    pos = open_pos + 1
    while pos < len(content) and depth > 0:
        if content[pos] == '{':
            depth += 1
        elif content[pos] == '}':
            depth -= 1
        pos += 1
    return pos


# ──────────────────────────────────────────────────────────────────────
#  C++ .cpp files — constructor injection, out-of-class methods, destructor
# ──────────────────────────────────────────────────────────────────────
def process_cpp(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    modified = False

    # ── 1. Add ros2top + nlohmann includes at end of first include block ──
    if '#include <ros2top/ros2top.hpp>' not in content:
        block_end = _find_include_block_end(content)
        if block_end > 0:
            content = content[:block_end] + '#include <ros2top/ros2top.hpp>\n#include <nlohmann/json.hpp>\n' + content[block_end:]
            modified = True

    # ── 2. Add <chrono> include if missing ───────────────────────────
    if '#include <chrono>' not in content:
        block_end = _find_include_block_end(content)
        if block_end > 0:
            content = content[:block_end] + '#include <chrono>\n' + content[block_end:]
            modified = True

    # ── 3. Add using directive if missing ────────────────────────────
    if 'using namespace std::chrono_literals' not in content:
        block_end = _find_include_block_end(content)
        if block_end > 0:
            content = content[:block_end] + '\nusing namespace std::chrono_literals;\n' + content[block_end:]
            modified = True

    # ── 4. Find constructors that initialise rclcpp::Node / LifecycleNode ──
    #   Step-by-step:
    #     a) find ClassName::ClassName(
    #     b) balance parens to close the parameter list
    #     c) find the body opening { (skipping brace-init members)
    #     d) check the initializer list for Node / LifecycleNode (word boundary)
    #     e) inject registration + heartbeat timer after the body {
    ctor_decl_re = re.compile(r'^([a-zA-Z0-9_]+)::(\1)\s*\(', re.MULTILINE)

    if 'ros2top::register_node' not in content:
        injected_classes = []
        # Restart search after each injection because offsets shift
        while True:
            made_change = False
            for ctor_match in ctor_decl_re.finditer(content):
                class_name = ctor_match.group(1)
                if class_name in injected_classes:
                    continue

                # Balance parens to find closing ) of parameter list
                paren_open = ctor_match.end() - 1
                depth = 1
                pos = paren_open + 1
                while pos < len(content) and depth > 0:
                    if content[pos] == '(':
                        depth += 1
                    elif content[pos] == ')':
                        depth -= 1
                    pos += 1
                paren_close = pos  # position after closing )

                # Find body opening brace (correctly skipping brace-init)
                body_brace = _find_body_brace(content, paren_close)
                if body_brace is None:
                    continue

                # Check initializer list for Node / LifecycleNode with word boundary
                # (avoids false positives like BT::ConditionNode, BT::ActionNode, etc.)
                init_section = content[paren_close:body_brace]
                if not re.search(r'(?<![a-zA-Z0-9_])(?:LifecycleNode|Node)\s*\(', init_section):
                    continue

                injection = (
                    "\n"
                    "  // --- ros2top registration ---\n"
                    "  ros2top_registered_ = false;\n"
                    "  try {\n"
                    "    nlohmann::json node_info_ros2top;\n"
                    '    node_info_ros2top["description"] = "Auto-registered nav2 node";\n'
                    f'    node_info_ros2top["node_type"] = "{class_name}";\n'
                    '    node_info_ros2top["language"] = "cpp";\n'
                    "    if (ros2top::register_node(this->get_name(), node_info_ros2top)) {\n"
                    "      ros2top_registered_ = true;\n"
                    "    }\n"
                    "  } catch (...) {}\n"
                    "  // ros2top heartbeat timer (every 5 s)\n"
                    "  ros2top_heartbeat_timer_ = this->create_wall_timer(\n"
                    f"      5000ms, std::bind(&{class_name}::ros2top_heartbeat_callback_, this));\n"
                    "  // --- end ros2top ---\n"
                )

                insert_pos = body_brace + 1
                content = content[:insert_pos] + injection + content[insert_pos:]
                injected_classes.append(class_name)
                modified = True
                made_change = True
                break  # restart iterator — offsets shifted

            if not made_change:
                break

    # ── 5. Inject out-of-class heartbeat + shutdown method implementations ──
    if 'ros2top::register_node' in content:
        class_names_with_ros2top = set()
        for m in re.finditer(r'std::bind\(&([a-zA-Z0-9_]+)::ros2top_heartbeat_callback_', content):
            class_names_with_ros2top.add(m.group(1))

        for class_name in class_names_with_ros2top:
            # Check for the actual method DEFINITION (with parentheses + body),
            # NOT just any reference like the std::bind call.
            if re.search(rf'void\s+{re.escape(class_name)}::ros2top_heartbeat_callback_\s*\(', content):
                continue  # Already defined

            # Find the constructor to insert methods after its closing brace
            ctor_re = re.compile(
                re.escape(class_name) + r'::' + re.escape(class_name) + r'\s*\(', re.DOTALL)
            ctor_match = ctor_re.search(content)
            if not ctor_match:
                continue

            body_brace = _find_body_brace(content, ctor_match.end())
            if body_brace is None:
                continue

            ctor_end = _find_matching_brace(content, body_brace)

            methods = (
                "\n"
                "// --- ros2top heartbeat and shutdown ---\n"
                "void\n"
                f"{class_name}::ros2top_heartbeat_callback_()\n"
                "{\n"
                "  if (ros2top_registered_) {\n"
                "    try {\n"
                "      ros2top::heartbeat(this->get_name());\n"
                "    } catch (...) {}\n"
                "  }\n"
                "}\n"
                "\n"
                "void\n"
                f"{class_name}::ros2top_shutdown_()\n"
                "{\n"
                "  if (ros2top_registered_) {\n"
                "    try {\n"
                "      ros2top::unregister_node(this->get_name());\n"
                "    } catch (...) {}\n"
                "    ros2top_registered_ = false;\n"
                "  }\n"
                "}\n"
                "// --- end ros2top heartbeat and shutdown ---\n"
            )
            content = content[:ctor_end] + '\n' + methods + content[ctor_end:]
            modified = True

        # ── 6. Inject ros2top_shutdown_() call into destructor / on_shutdown / on_cleanup ──
        for class_name in class_names_with_ros2top:
            shutdown_call = 'ros2top_shutdown_();'
            if shutdown_call in content:
                continue  # Already present for some class (single-class-per-file assumption)

            # a) Try existing destructor
            dtor_re = re.compile(
                r'(' + re.escape(class_name) + r'::~' + re.escape(class_name) + r'\s*\(\s*\)\s*\{)',
                re.MULTILINE)
            dtor_match = dtor_re.search(content)
            if dtor_match:
                insert_pos = dtor_match.end()
                content = content[:insert_pos] + '\n  ' + shutdown_call + '\n' + content[insert_pos:]
                modified = True
                continue

            # b) Try on_shutdown (LifecycleNode)
            on_shutdown_re = re.compile(
                r'(' + re.escape(class_name) + r'::on_shutdown\s*\([^)]*\)\s*\{)',
                re.MULTILINE)
            on_shutdown_match = on_shutdown_re.search(content)
            if on_shutdown_match:
                insert_pos = on_shutdown_match.end()
                content = content[:insert_pos] + '\n  ' + shutdown_call + '\n' + content[insert_pos:]
                modified = True
                continue

            # c) Try on_cleanup (LifecycleNode)
            on_cleanup_re = re.compile(
                r'(' + re.escape(class_name) + r'::on_cleanup\s*\([^)]*\)\s*\{)',
                re.MULTILINE)
            on_cleanup_match = on_cleanup_re.search(content)
            if on_cleanup_match:
                insert_pos = on_cleanup_match.end()
                content = content[:insert_pos] + '\n  ' + shutdown_call + '\n' + content[insert_pos:]
                modified = True
                continue

    if modified:
        with open(filepath, 'w') as f:
            f.write(content)
        return True
    return False


# ──────────────────────────────────────────────────────────────────────
#  C++ .hpp headers — member variable and method declarations in class
# ──────────────────────────────────────────────────────────────────────
def process_hpp(filepath):
    """Inject ros2top member variables and method declarations into class headers."""
    with open(filepath, 'r') as f:
        content = f.read()

    # Only process headers that declare Node/LifecycleNode subclasses
    if not re.search(r'class\s+\w+\s*:\s*public\s+(?:\w+::)*(?:LifecycleNode|Node)', content):
        return False

    modified = False

    # Add ros2top include if not present (at end of first include block)
    if '#include <ros2top/ros2top.hpp>' not in content:
        block_end = _find_include_block_end(content)
        if block_end > 0:
            insertion = '#include <ros2top/ros2top.hpp>\n#include <nlohmann/json.hpp>\n'
            content = content[:block_end] + insertion + content[block_end:]
            modified = True

    # Skip if already has ros2top members
    if 'ros2top_registered_' in content:
        return False

    # Find class declarations that inherit from Node/LifecycleNode
    class_pattern = re.compile(
        r'(class\s+(\w+)\s*:\s*public\s+(?:\w+::)*(?:LifecycleNode|Node)\s*\{)',
        re.DOTALL
    )

    for class_match in class_pattern.finditer(content):
        class_name = class_match.group(2)

        # Find the matching closing }; for this class
        brace_start = class_match.end() - 1
        depth = 1
        pos = brace_start + 1
        while pos < len(content) and depth > 0:
            if content[pos] == '{':
                depth += 1
            elif content[pos] == '}':
                depth -= 1
            pos += 1
        closing_brace_pos = pos - 1  # position of }

        member_injection = f"""
  // --- ros2top members ---
  void ros2top_heartbeat_callback_();
  void ros2top_shutdown_();
  rclcpp::TimerBase::SharedPtr ros2top_heartbeat_timer_;
  bool ros2top_registered_{{false}};
  // --- end ros2top members ---
"""
        content = content[:closing_brace_pos] + member_injection + content[closing_brace_pos:]
        modified = True
        break  # Only inject into the first matching class per header

    if modified:
        with open(filepath, 'w') as f:
            f.write(content)
        return True
    return False


# ──────────────────────────────────────────────────────────────────────
#  Python nodes
# ──────────────────────────────────────────────────────────────────────
def process_python(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # Only process files that reference rclpy Node
    if 'Node' not in content or 'rclpy' not in content:
        return False

    modified = False

    # ── 1. Add / update import ───────────────────────────────────────
    full_import = 'from ros2top.node_registry import register_node, unregister_node, heartbeat'

    if 'from ros2top.node_registry import register_node' in content:
        if 'unregister_node' not in content or 'heartbeat' not in content:
            content = re.sub(
                r'from ros2top\.node_registry import register_node[^\n]*',
                full_import, content)
            modified = True
    elif 'from ros2top.node_registry' not in content and 'register_node' not in content:
        import_block = (
            "try:\n"
            "    from ros2top.node_registry import register_node, unregister_node, heartbeat\n"
            "    ROS2TOP_AVAILABLE = True\n"
            "except ImportError:\n"
            "    ROS2TOP_AVAILABLE = False\n"
        )
        import_positions = []
        for m in re.finditer(r'^(?:import\s+\w+|from\s+\w+[\.\w]*\s+import\s+\w+)[^\n]*\n', content, re.MULTILINE):
            import_positions.append(m.end())
        if import_positions:
            insert_pos = import_positions[-1]
            content = content[:insert_pos] + '\n' + import_block + '\n' + content[insert_pos:]
            modified = True

    # ── 2. Find __init__ in Node subclasses and inject registration ──
    init_super_pattern = re.compile(
        r'(def\s+__init__\([^)]*\):\s*\n(?:\s+[^\n]*\n)*?\s*super\(\s*(?:[^)]*)\s*\)\.__init__\([^)]*\))',
        re.MULTILINE
    )

    def python_init_replacer(match):
        prefix = match.group(0)

        lines = prefix.split('\n')
        super_line = [l for l in lines if 'super(' in l]
        if super_line:
            indent = re.match(r'^(\s*)', super_line[0]).group(1)
        else:
            indent = '        '

        injection = (
            f"\n{indent}# --- ros2top registration ---\n"
            f"{indent}self.ros2top_registered = False\n"
            f"{indent}try:\n"
            f"{indent}    register_node(self.get_name(), {{\n"
            f'{indent}        "description": "Auto-registered nav2 node",\n'
            f'{indent}        "node_type": self.__class__.__name__,\n'
            f'{indent}        "language": "python"\n'
            f"{indent}    }})\n"
            f"{indent}    self.ros2top_registered = True\n"
            f"{indent}except Exception:\n"
            f"{indent}    pass\n"
            f"{indent}self.ros2top_heartbeat_timer = self.create_timer(\n"
            f"{indent}    5.0, self._ros2top_heartbeat_callback)\n"
            f"{indent}# --- end ros2top ---"
        )
        return prefix + injection

    if 'register_node(self.get_name()' not in content and 'register_node(' not in content:
        new_content = init_super_pattern.sub(python_init_replacer, content)
        if new_content != content:
            content = new_content
            modified = True

    # ── 3. Inject heartbeat callback and shutdown methods ────────────
    # Check for the method DEFINITION, not just any reference to the name
    if ('ros2top_heartbeat_timer' in content or 'ros2top_registered' in content) and \
       'def _ros2top_heartbeat_callback(self)' not in content:
        class_pattern = re.compile(
            r'class\s+(\w+)\s*\([^)]*\bNode\b[^)]*\)\s*:', re.MULTILINE)

        for class_match in class_pattern.finditer(content):
            class_start = class_match.end()
            method_match = re.search(r'^(\s+)def\s+', content[class_start:], re.MULTILINE)
            if method_match:
                indent = method_match.group(1)
            else:
                indent = '    '

            next_class = re.search(r'^class\s+', content[class_start:], re.MULTILINE)
            if next_class:
                insert_pos = class_start + next_class.start()
            else:
                next_func = re.search(r'^def\s+', content[class_start:], re.MULTILINE)
                if next_func:
                    insert_pos = class_start + next_func.start()
                else:
                    insert_pos = len(content)

            methods = (
                f"\n{indent}def _ros2top_heartbeat_callback(self):\n"
                f"{indent}    \"\"\"Send heartbeat to ros2top.\"\"\"\n"
                f"{indent}    if self.ros2top_registered:\n"
                f"{indent}        try:\n"
                f"{indent}            heartbeat(self.get_name())\n"
                f"{indent}        except Exception:\n"
                f"{indent}            pass\n"
                f"\n"
                f"{indent}def _ros2top_shutdown(self):\n"
                f"{indent}    \"\"\"Unregister from ros2top on shutdown.\"\"\"\n"
                f"{indent}    if self.ros2top_registered:\n"
                f"{indent}        try:\n"
                f"{indent}            unregister_node(self.get_name())\n"
                f"{indent}            self.ros2top_registered = False\n"
                f"{indent}        except Exception:\n"
                f"{indent}            pass\n"
                f"\n"
            )
            content = content[:insert_pos] + methods + content[insert_pos:]
            modified = True
            break

    if modified:
        with open(filepath, 'w') as f:
            f.write(content)
        return True
    return False


# ──────────────────────────────────────────────────────────────────────
#  Main entry point
# ──────────────────────────────────────────────────────────────────────
def main(workspace_dir):
    for root, dirs, files in os.walk(workspace_dir):
        # Skip build / install / log / hidden / test directories
        dirs[:] = [d for d in dirs if d not in (
            'build', 'install', 'log', '.git', '__pycache__',
            'test', 'tests', 'test_utils',
        )]

        # Also skip entire test packages (e.g. nav2_system_tests)
        pkg_name = os.path.basename(root)
        if 'test' in pkg_name.lower():
            dirs[:] = []
            continue

        for file in files:
            filepath = os.path.join(root, file)

            try:
                if file == 'package.xml':
                    if process_package_xml(filepath):
                        print(f"Modified {filepath}")
                elif file == 'CMakeLists.txt':
                    if process_cmakelists(filepath):
                        print(f"Modified {filepath}")
                elif file.endswith('.cpp'):
                    if process_cpp(filepath):
                        print(f"Modified {filepath}")
                elif file.endswith('.hpp'):
                    if process_hpp(filepath):
                        print(f"Modified {filepath}")
                elif file.endswith('.py') and not file.endswith('setup.py'):
                    if process_python(filepath):
                        print(f"Modified {filepath}")
            except Exception as e:
                print(f"Failed processing {filepath}: {e}")


if __name__ == '__main__':
    workspace_dir = sys.argv[1] if len(sys.argv) > 1 else '.'
    main(workspace_dir)
