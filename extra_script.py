Import("env")

def post_build(source, target, env):
    elf_file = str(target[0])
    lst_file = elf_file.replace(".elf", ".lst")
    # 执行反汇编命令生成 .lst 文件
    env.Execute("arm-none-eabi-objdump -h -S " + elf_file + " > " + lst_file)

env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", post_build)
