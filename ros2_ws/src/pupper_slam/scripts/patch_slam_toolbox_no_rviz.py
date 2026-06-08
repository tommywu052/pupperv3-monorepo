"""Patch slam_toolbox to skip RViz plugin target (Pi lacks tf2_ros dev headers)."""

import re
from pathlib import Path

CMAKE = Path("CMakeLists.txt")
text = CMAKE.read_text(encoding="utf-8")

if "option(BUILD_RVIZ_PLUGIN" in text:
    print("Already patched")
    raise SystemExit(0)

text = text.replace(
    "project(slam_toolbox)\n\nset(CMAKE_BUILD_TYPE Release)",
    "project(slam_toolbox)\n\noption(BUILD_RVIZ_PLUGIN \"Build RViz plugin\" OFF)\n\nset(CMAKE_BUILD_TYPE Release)",
    1,
)

text = re.sub(
    r"set\(libraries\n\s+toolbox_common\n\s+SlamToolboxPlugin\n",
    "set(libraries\n    toolbox_common\n",
    text,
    count=1,
)

rviz_start = "#### rviz Plugin"
rviz_end = "#### Ceres Plugin"
i0 = text.index(rviz_start)
i1 = text.index(rviz_end)
block = text[i0:i1]
text = text[:i0] + f"if(BUILD_RVIZ_PLUGIN)\n{block}endif()\n\n" + text[i1:]

text = re.sub(
    r"install\(TARGETS SlamToolboxPlugin[\s\S]*?INCLUDES DESTINATION include\n\)\n",
    lambda m: f"if(BUILD_RVIZ_PLUGIN)\n{m.group(0)}endif()\n",
    text,
    count=1,
)

text = text.replace(
    "ament_export_targets(SlamToolboxPlugin HAS_LIBRARY_TARGET)\nament_package()",
    "if(BUILD_RVIZ_PLUGIN)\nament_export_targets(SlamToolboxPlugin HAS_LIBRARY_TARGET)\nendif()\nament_package()",
    1,
)

CMAKE.write_text(text, encoding="utf-8")
print("Patched CMakeLists.txt (BUILD_RVIZ_PLUGIN=OFF)")
