# Skills 共享配置

## 目的

让 `.lingma` 和 `.claude` 共用同一套 skills 配置。

## 创建符号链接

### Windows

```cmd
cd D:\UEProject\uav_simulator\.lingma
mklink /D skills ..\\.claude\\skills
```

需要管理员权限或开启开发者模式。

### macOS/Linux

```bash
cd /path/to/uav_simulator/.lingma
ln -s ../.claude/skills skills
```

## Git 同步

```bash
# 提交符号链接
git add .lingma/skills
git commit -m "Add skills symlink"

# Windows 需确保启用符号链接支持
git config core.symlinks true
```

只需在一个平台创建，其他平台 `git pull` 后自动生效。
