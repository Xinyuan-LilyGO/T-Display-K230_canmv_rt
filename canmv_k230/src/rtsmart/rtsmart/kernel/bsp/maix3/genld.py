import os
import rtconfig
import platform
import subprocess

def genld(input, output):
    if rtconfig.PLATFORM == 'gcc':

        gcc_cmd = os.path.join(rtconfig.EXEC_PATH, rtconfig.CC)

        # gcc -E -P -x c script.ld -o expanded_script.ld
        if(platform.system() == 'Windows'):
            child = subprocess.Popen([gcc_cmd, '-E', '-P', '-x', 'c', input, '-o', output], stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
        else:
            child = subprocess.Popen(gcc_cmd + f' -E -P -x c {input} -o {output}', stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)

        stdout, stderr = child.communicate()

        print(stdout)
        if stderr != '':
            print(stderr)

        print('Generate ld done.')
