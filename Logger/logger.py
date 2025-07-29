import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import serial
import serial.tools.list_ports
import threading
import time
from datetime import datetime
import os


logging_active = [False]

def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

def start_logging():
    port = port_var.get()
    baud = int(baud_var.get())
    if not port or not baud:
        messagebox.showerror("Error", "Seleccioná un puerto y baudrate.")
        return

    folder = folder_path.get()
    if not os.path.isdir(folder):
        messagebox.showerror("Error", "Carpeta inválida. Seleccioná una carpeta de guardado.")
        return

    # Crear nombre automático
    filename = f"log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    output_path = os.path.join(folder, filename)

    def read_serial():
        try:
            ser = serial.Serial(port, baud)
            time.sleep(2)  # Estabilizar conexión
            with open(output_path, 'w') as f:
                f.write("Timestamp,Datos\n")
                while logging_active[0]:
                    if ser.in_waiting:
                        line = ser.readline().decode(errors="ignore").strip()
                        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                        full_line = f"{timestamp},{line}"
                        log_box.insert(tk.END, full_line + '\n')
                        log_box.see(tk.END)
                        f.write(full_line + '\n')
        except Exception as e:
            messagebox.showerror("Error", str(e))

    logging_active[0] = True
    thread = threading.Thread(target=read_serial)
    thread.start()

def stop_logging():
    logging_active[0] = False

def refresh_ports():
    ports = list_serial_ports()
    port_menu['values'] = ports
    if ports:
        port_var.set(ports[0])

# ------------------- GUI -------------------

root = tk.Tk()
root.title("Serial Logger CSV")

# Puerto COM
tk.Label(root, text="Puerto COM").pack()
port_var = tk.StringVar()
port_menu = ttk.Combobox(root, textvariable=port_var, state="readonly")
port_menu.pack()
refresh_ports()

# Baudrate
tk.Label(root, text="Baudrate").pack()
baud_var = tk.StringVar()
baud_menu = ttk.Combobox(root, textvariable=baud_var, state="readonly")
baud_menu['values'] = ["9600", "19200", "38400", "57600", "115200"]
baud_menu.set("9600")
baud_menu.pack()

# Botones
btn_frame = tk.Frame(root)
btn_frame.pack(pady=5)

# Carpeta destino
tk.Label(root, text="Carpeta de guardado").pack()
folder_frame = tk.Frame(root)
folder_frame.pack()

folder_path = tk.StringVar()
folder_entry = tk.Entry(folder_frame, textvariable=folder_path, width=40)
folder_entry.pack(side=tk.LEFT)

def browse_folder():
    path = filedialog.askdirectory(title="Seleccionar carpeta de guardado")
    if path:
        folder_path.set(path)

tk.Button(folder_frame, text="📁", command=browse_folder).pack(side=tk.LEFT)

tk.Button(btn_frame, text="Iniciar Log", command=start_logging).grid(row=0, column=0, padx=5)
tk.Button(btn_frame, text="Detener Log", command=stop_logging).grid(row=0, column=1, padx=5)
tk.Button(btn_frame, text="Actualizar Puertos", command=refresh_ports).grid(row=0, column=2, padx=5)

# Área de texto
log_box = tk.Text(root, height=15, width=60)
log_box.pack()

root.mainloop()
