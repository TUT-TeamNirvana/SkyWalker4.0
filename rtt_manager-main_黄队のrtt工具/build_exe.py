"""
RTT Manager EXE打包脚本
"""
import PyInstaller.__main__
import sys
import os

def build_exe():
    # 获取项目根目录
    project_dir = os.path.dirname(os.path.abspath(__file__))
    
    # PyInstaller参数
    args = [
        'rtt_manager.py',
        '--onefile',                    # 生成单个EXE文件
        '--windowed',                   # 隐藏控制台窗口
        '--name=RTT_Manager',           # EXE名称
        '--add-data=config.json:.',     # 包含config.json文件
        f'--distpath={project_dir}/dist',   # 输出目录
        f'--workpath={project_dir}/build',  # 构建目录
        f'--specpath={project_dir}',        # spec文件目录
    ]
    
    print("开始打包...")
    print(f"项目目录: {project_dir}")
    print(f"参数: {' '.join(args)}")
    
    PyInstaller.__main__.run(args)
    
    print("\n" + "="*50)
    print("打包完成！")
    print(f"EXE文件位置: {project_dir}/dist/RTT_Manager.exe")
    print("="*50)

if __name__ == '__main__':
    build_exe()
