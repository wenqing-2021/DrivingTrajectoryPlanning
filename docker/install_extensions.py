# user: shijie.yuan
# email: yuansj@hnu.edu.cn
# time: 2023-10-24

import os
import json
import re

extension_list = [
    "ms-vscode.cpptools",
    "llvm-vs-code-extensions.vscode-clangd",
    "ms-vscode.cmake-tools",
    "eamodio.gitlens",
    "mhutchie.git-graph",
    "ms-python.python",
    "ms-python.black-formatter",
    "GitHub.copilot",
]

SETTING_CONFIG = {"terminal.integrated.defaultProfile.linux": "zsh"}


def install_extensions():
    run_file = "docker/scripts/install_extensions.sh"
    os.system(f"chmod 777 {run_file}")
    params = ""
    for extension_name in extension_list:
        params += " " + extension_name

    os.system(run_file + params)
    if not os.path.exists(".vscode"):
        os.mkdir(".vscode")
    if not os.path.exists(".vscode/settings.json"):
        os.system("touch .vscode/settings.json")
        with open(".vscode/settings.json", "w") as f:
            json.dump(SETTING_CONFIG, f, indent=4)
    else:
        with open(".vscode/settings.json", "r") as f:
            setting_config = json.load(f)
        setting_config.update(SETTING_CONFIG)
        with open(".vscode/settings.json", "w") as f:
            json.dump(setting_config, f, indent=4)

    print("install powerlevel10k")
    if not os.path.exists("/root/.oh-my-zsh/custom/themes/powerlevel10k"):
        os.system(
            "git clone --depth=1 https://gitee.com/romkatv/powerlevel10k.git ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k"
        )

    # 1. 打开文件以进行读取
    file_path = "/root/.zshrc"  # 替换为要修改的文件路径
    with open(file_path, "r") as file:
        content = file.read()

    # 2. 使用正则表达式搜索要替换的位置
    pattern = r'ZSH_THEME=".+?"'  # 正则表达式模式，匹配 ZSH_THEME="任何内容"
    match = re.search(pattern, content)

    if match:
        # 找到匹配的文本
        old_text = match.group()
        new_text = 'ZSH_THEME="powerlevel10k/powerlevel10k"'
        content = content.replace(old_text, new_text)

        # 3. 打开文件以进行写入
        with open(file_path, "w") as file:
            # 4. 将编辑后的内容写入文件
            file.write(content)
        print("has replace zsh_theme with powerlevel10k")
    else:
        print("无法找到要替换的zsh_theme")

    # find plugins=(git)
    plugins_pattern = r"plugins=\(git\)"  # 正则表达式模式，匹配 plugins=(git)
    match_plugins = re.search(plugins_pattern, content)
    if match_plugins:
        # 找到匹配的文本
        old_text = match_plugins.group()
        new_text = "plugins=(git zsh-autosuggestions zsh-syntax-highlighting)"
        content = content.replace(old_text, new_text)

        # 3. 打开文件以进行写入
        with open(file_path, "w") as file:
            # 4. 将编辑后的内容写入文件
            file.write(content)
        print(
            "has replace plugins with git zsh-autosuggestions zsh-syntax-highlighting"
        )
    else:
        print("无法找到要替换的plugins")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="install vscode extension")
    parser.add_argument(
        "--install", "-i", action="store_true", help="install extension"
    )
    args = parser.parse_args()
    if args.install:
        install_extensions()
