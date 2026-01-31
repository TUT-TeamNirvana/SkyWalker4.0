import tkinter as tk
from tkinter import ttk, messagebox
import json, subprocess, threading, os, sys

class RTTManager:
    def __init__(self, root):
        self.root = root
        self.root.title("RTT 启动器")
        
        # 窗口居中
        w, h = 400, 350
        self.root.geometry(f"{w}x{h}+{int((self.root.winfo_screenwidth()-w)/2)}+{int((self.root.winfo_screenheight()-h)/2)}")
        
        # 获取config.json的完整路径 (同一目录下)
        if getattr(sys, 'frozen', False):
            # 打包后的EXE环境
            self.base_dir = os.path.dirname(sys.executable)
        else:
            # 开发环境
            self.base_dir = os.path.dirname(os.path.abspath(__file__))
        
        self.config_file = os.path.join(self.base_dir, 'config.json')
        self.process = None
        self.config = self.load_config()

        # --- UI 界面 ---
        tk.Label(root, text="选择调试器:", font=("微软雅黑", 9)).pack(pady=5)
        self.if_var = tk.StringVar()
        self.if_combo = ttk.Combobox(root, textvariable=self.if_var, state="readonly")
        self.if_combo.pack(padx=20, fill='x')

        tk.Label(root, text="选择芯片:", font=("微软雅黑", 9)).pack(pady=5)
        self.chip_var = tk.StringVar()
        self.chip_combo = ttk.Combobox(root, textvariable=self.chip_var, state="readonly")
        self.chip_combo.pack(padx=20, fill='x')

        self.update_ui_from_config()

        self.btn_start = tk.Button(root, text="启动服务", command=self.toggle_service, bg="#4CAF50", fg="white", font=("微软雅黑", 10, "bold"))
        self.btn_start.pack(pady=15, ipadx=20)

        self.log_text = tk.Text(root, height=10, font=("Consolas", 8), bg="#f0f0f0")
        self.log_text.pack(padx=10, pady=5, fill='both', expand=True)

    def load_config(self):
        try:
            if os.path.exists(self.config_file):
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    # 确保格式正确
                    if "interface" in data and "chips" in data:
                        return data
        except: pass
        return {"interface": [], "chips": []}

    def update_ui_from_config(self):
        self.if_combo['values'] = [i['name'] for i in self.config.get('interface', [])]
        if self.if_combo['values']: self.if_combo.current(0)
        self.chip_combo['values'] = [c['name'] for c in self.config.get('chips', [])]
        if self.chip_combo['values']: self.chip_combo.current(0)

    def toggle_service(self):
        if self.process is None:
            # ... (保持启动逻辑不变)
            # 1. 获取 UI 选中的配置
            if_cfg = next(i['config'] for i in self.config['interface'] if i['name'] == self.if_var.get())
            chip = next(c for c in self.config['chips'] if c['name'] == self.chip_var.get())
            
            # 2. 读取并计算地址与时钟
            start_addr = chip['ram_start']
            end_addr = int(chip['ram_end'], 16)
            scan_size = end_addr - int(start_addr, 16)
            swd_clock = int(chip.get('clock', '2000'))
            
            # 获取缓冲区大小，默认给 1024 字节
            buf_size = int(chip.get('rtt_ram_size', '1024'))

            #获取缓冲区通道
            channel = int(chip.get('channel', '0'))

            # 算法逻辑：
            # 1. 计算排空缓冲区所需时间 (ms) = (位宽 / 速率) / 效率
            # 2. 轮询间隔应小于排空时间的一半以保证安全
            # 3. 效率因子取 0.6 (SWD 开销)
            calc_interval = (buf_size * 8) / (swd_clock * 0.6) * 0.5
            
            # 约束区间：最低 2ms (保护 CPU)，最高 50ms (保证波形流畅)
            final_polling = max(2, min(50, int(calc_interval)))
            # ---------------------------------------

            # 3. 构造指令
            cmd = [
                "openocd", "-f", if_cfg,  
                "-c", "set CPUTAPID 0", 
                "-f", chip['target'],
                "-c", f"adapter speed {swd_clock}", 
                "-c", "init",
                "-c", f"rtt setup {start_addr} {hex(scan_size)} \"SEGGER RTT\"",
                "-c", f"rtt polling_interval {final_polling}",
                "-c", "rtt start",
                "-c", f"rtt server start 60001 {channel}"
            ]
            
            self.log_text.delete(1.0, tk.END)
            # 在日志首行静默告知计算结果
            from datetime import datetime
            start_time = datetime.now().strftime("%H:%M:%S")
            log_msg = f"[{start_time}] >>> 自动策略: SWD {swd_clock}K, Buf {buf_size}B -> Polling {final_polling}ms\n"
            self.log_text.insert(tk.END, log_msg)
            self.log_text.insert(tk.END, "-"*60 + "\n")

            self.process = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, 
                text=True, creationflags=subprocess.CREATE_NO_WINDOW
            )
            threading.Thread(target=self.monitor, daemon=True).start()
            self.btn_start.config(text="停止服务", bg="#f44336")
        else:
            # --- 停止逻辑 ---
            if self.process:
                self.process.terminate()
                self.process = None
                
                # 导入时间模块并输出日志
                from datetime import datetime
                current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                stop_msg = f"\n----------------------------------------\n"
                stop_msg += f"[{current_time}] Info : OpenOCD 服务已停止。\n"
                
                self.log_text.insert(tk.END, stop_msg)
                self.log_text.see(tk.END) # 自动滚动到底部
            
            self.btn_start.config(text="启动服务", bg="#4CAF50")
    def on_closing(self):
        #当用户点击窗口右上角 X 时执行
        if self.process:
            # 如果服务正在运行，先停止它
            self.stop_service_logic()
        
        # 正式销毁窗口
        self.root.destroy()
    def monitor(self):
        for line in iter(self.process.stdout.readline, ''):
            self.log_text.insert(tk.END, line)
            self.log_text.see(tk.END)

if __name__ == "__main__":
    root = tk.Tk()
    RTTManager(root)
    root.mainloop()