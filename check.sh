#!/bin/bash
# 项目完整性检查脚本

echo "=========================================="
echo "项目完整性检查"
echo "=========================================="
echo ""

# 检查必需文件
files=(
    "index.html"
    "css/style.css"
    "js/main.js"
    "js/pointcloud.js"
    "js/roi_selector.js"
    "launch/webui.launch"
    "test/test_publisher.py"
    "README.md"
)

missing=0
for file in "${files[@]}"; do
    if [ -f "$file" ]; then
        size=$(wc -c < "$file")
        echo "✓ $file ($size bytes)"
    else
        echo "✗ $file (缺失)"
        missing=$((missing + 1))
    fi
done

echo ""
if [ $missing -eq 0 ]; then
    echo "✓ 所有文件完整"
    echo ""
    echo "项目统计:"
    echo "- HTML 文件: $(find . -name "*.html" | wc -l)"
    echo "- CSS 文件: $(find . -name "*.css" | wc -l)"
    echo "- JavaScript 文件: $(find . -name "*.js" | wc -l)"
    echo "- Python 文件: $(find . -name "*.py" | wc -l)"
    echo "- Launch 文件: $(find . -name "*.launch" | wc -l)"
    echo ""
    echo "JavaScript 代码行数:"
    wc -l js/*.js
else
    echo "✗ 缺失 $missing 个文件"
    exit 1
fi
