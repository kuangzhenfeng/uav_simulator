
# clangd 配置方法

## 1. 安装 clangd
使用visual studio installer安装clang工具集

## 2.配置 VSCode 工作区设置（.code-workspace）
```json
{
    "settings": {
        "C_Cpp.intelliSenseEngine": "disabled",
        "clangd.arguments": [
            "--background-index",
            "--clang-tidy",
            "--query-driver=D:\\mySoftware\\Microsoft Visual Studio\\*\\Professional\\VC\\Tools\\MSVC\\*\\bin\\Hostx64\\x64\\cl.exe;D:\\mySoftware\\Microsoft Visual Studio\\2022\\Professional\\VC\\Tools\\Llvm\\x64\\bin\\clang-cl.exe"
        ],
        "clangd.path": "D:\\mySoftware\\Microsoft Visual Studio\\2022\\Professional\\VC\\Tools\\Llvm\\x64\\bin\\clangd.exe"
    }
}
```

## 3.生成clangd用的compile_commands.json文件
```shell
.\Script\generate_clang_database.bat
```
