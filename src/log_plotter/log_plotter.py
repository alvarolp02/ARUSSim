import argparse
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import numpy as np
import pandas as pd
import tkinter as tk
from tkinter import ttk, StringVar, filedialog, messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import re
import sys
import array


def read_ros_messages(input_bag: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    topic_types = reader.get_all_topics_and_types()

    def typename(topic_name):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_type = get_message(typename(topic))
        msg = deserialize_message(data, msg_type)
        yield topic, msg, timestamp, msg_type
    del reader


def to_safe_identifier(name):
    return "_" + re.sub(r"[^a-zA-Z0-9_]", "_", name)


def plot_variables(df):
    root = tk.Tk()
    root.title("Log Plotter")

    main_frame = tk.Frame(root)
    main_frame.pack(side="left", fill="y", padx=5, pady=5)

    canvas_frame = tk.Frame(root)
    canvas_frame.pack(side="right", fill="both", expand=True, padx=5, pady=5)

    tk.Label(main_frame, text="X Axis:").pack(pady=(5, 0))
    x_axis_var = StringVar(value="timestamp")
    x_selector = ttk.Combobox(main_frame, textvariable=x_axis_var, state="readonly")
    x_selector.pack(fill="x", padx=5)


    # Field for expressions
    tk.Label(main_frame, text="Expression (e.g.: /a.x + /b.y):").pack(pady=(10, 0))
    expr_entry = tk.Entry(main_frame)
    expr_entry.pack(fill="x", padx=5)
    expr_btn = tk.Button(main_frame, text="Add Expression")
    expr_btn.pack(pady=(2, 5))

    tk.Label(main_frame, text="Variables by topic:").pack(pady=(10, 0))

    scroll_canvas = tk.Canvas(main_frame)
    scrollbar = ttk.Scrollbar(main_frame, orient="vertical", command=scroll_canvas.yview)
    topics_container = tk.Frame(scroll_canvas)
    

    topics_container.bind(
        "<Configure>",
        lambda e: scroll_canvas.configure(scrollregion=scroll_canvas.bbox("all"))
    )

    scroll_canvas.create_window((0, 0), window=topics_container, anchor="nw")
    scroll_canvas.configure(yscrollcommand=scrollbar.set)
    scroll_canvas.pack(side="left", fill="both", expand=True)
    scrollbar.pack(side="right", fill="y")

    def _on_mousewheel(event):
        scroll_canvas.yview_scroll(int(-1*(event.delta/120)), "units")

    # Windows & macOS (delta-based scrolling)
    scroll_canvas.bind_all("<MouseWheel>", _on_mousewheel)

    # Linux (button-based scrolling)
    scroll_canvas.bind_all("<Button-4>", lambda e: scroll_canvas.yview_scroll(-1, "units"))
    scroll_canvas.bind_all("<Button-5>", lambda e: scroll_canvas.yview_scroll(1, "units"))


    

    # Group columns by topic
    topic_fields = {}
    for col in df.columns:
        if col == "timestamp":
            continue
        if '.' in col:
            topic, field = col.split('.', 1)
        else:
            topic, field = col, ""

        if topic not in topic_fields:
            topic_fields[topic] = []
        topic_fields[topic].append((col, field))

    x_options = [col for col in df.columns if col != "timestamp" and pd.api.types.is_numeric_dtype(df[col])]
    x_selector["values"] = ["timestamp"] + sorted(x_options)

    check_vars = {}
    expr_counter = [0]

    fig = Figure(figsize=(7, 5), dpi=100)
    ax = fig.add_subplot(111)
    canvas = FigureCanvasTkAgg(fig, master=canvas_frame)
    canvas_widget = canvas.get_tk_widget()
    canvas_widget.pack(fill="both", expand=True)

    toolbar = NavigationToolbar2Tk(canvas, canvas_frame)
    toolbar.update()
    toolbar.pack(side="top", fill="x")

    def on_selection_change():
        selected = [col for col, var in check_vars.items() if var.get()]
        ax.clear()

        x_col = x_axis_var.get()
        if x_col not in df.columns:
            x_col = "timestamp"

        if selected:
            for col in selected:
                if x_col == "timestamp":
                    ax.plot(df[x_col], df[col], label=col)
                else:
                    ax.scatter(df[x_col], df[col], s=10, label=col)

        ax.set_xlabel(x_col)
        ax.set_ylabel("Value")
        ax.grid(True)
        ax.legend()
        canvas.draw()

    x_selector.bind("<<ComboboxSelected>>", lambda e: on_selection_change())

    def toggle_frame(frame, button):
        def toggler():
            if frame.winfo_viewable():
                frame.pack_forget()
                button.config(text=button.cget("text").replace("▼", "▶"))
            else:
                frame.pack(fill="x", padx=20)
                button.config(text=button.cget("text").replace("▶", "▼"))
        return toggler

    for topic, fields in sorted(topic_fields.items()):
        topic_frame = tk.Frame(topics_container)
        topic_frame.pack(fill="x", pady=3)

        if len(fields) == 1:
            full_col, field = fields[0]
            var = tk.BooleanVar()
            var.trace_add("write", lambda *_: on_selection_change())
            if len(field) > 0:
                label = f"{topic}.{field}"
            else:
                label = topic
            check = tk.Checkbutton(topic_frame, text=label, variable=var)
            check.pack(anchor="w", padx=10)
            check_vars[full_col] = var


        else:
            btn = tk.Button(topic_frame, text=f"▶ {topic}", anchor="w", relief="flat")
            btn.pack(fill="x")

            fields_frame = tk.Frame(topic_frame)
            btn.config(command=toggle_frame(fields_frame, btn))

            for full_col, field in fields:
                var = tk.BooleanVar()
                var.trace_add("write", lambda *_: on_selection_change())
                check = tk.Checkbutton(fields_frame, text=field, variable=var)
                check.pack(anchor="w")
                check_vars[full_col] = var


    def add_expression():
        expr = expr_entry.get().strip()
        if not expr:
            return

        try:
            safe_cols = {to_safe_identifier(col): df[col] for col in df.columns}
            safe_expr = expr
            for col in df.columns:
                safe_expr = safe_expr.replace(col, to_safe_identifier(col))

            result = eval(safe_expr, {"__builtins__": {}}, safe_cols)
            print(safe_expr)
            print(result)

            col_name = f"expr_{expr_counter[0]}"
            df[col_name] = result
            expr_counter[0] += 1

            # Add checkbox
            var = tk.BooleanVar(value=True)
            var.trace_add("write", lambda *_: on_selection_change())
            row_frame = tk.Frame(topics_container)
            row_frame.pack(fill="x", anchor="w", padx=10, pady=1)

            check = tk.Checkbutton(row_frame, text=f"{col_name} ({expr})", variable=var)
            check.pack(side="left", anchor="w")
            check_vars[col_name] = var
            on_selection_change()

            def delete_column():
                row_frame.destroy()
                if col_name in df.columns:
                    df.drop(columns=[col_name], inplace=True)
                check_vars.pop(col_name, None)
                x_vals = list(x_selector["values"])
                if col_name in x_vals:
                    x_vals.remove(col_name)
                    x_selector["values"] = x_vals
                    if x_axis_var.get() == col_name:
                        x_axis_var.set("timestamp")
                on_selection_change()

            del_btn = tk.Button(row_frame, text="Delete", command=delete_column)
            del_btn.pack(side="right", padx=5)


            # Add to X axis combo
            current_x = list(x_selector["values"])
            if col_name not in current_x:
                x_selector["values"] = current_x + [col_name]
        except Exception as e:
            print(f"Error in expression: {e}")

    expr_btn.config(command=add_expression)

    root.mainloop()


def read_rosbag_mcap(file_path: str):
    rows = []
    seen_columns = set()

    for topic, msg, timestamp, msg_type in read_ros_messages(file_path):
        ts_sec = timestamp * 1e-9
        row = {"timestamp": ts_sec}

        for field in msg.get_fields_and_field_types():
            if field == "header":
                continue

            value = getattr(msg, field)        
            if isinstance(value, (int, float)):
                column_name = f"{topic}.{field}"
                row[column_name] = value
                seen_columns.add(column_name)
            elif isinstance(value, (array.array)) and len(value)<=20:
                for i, val in enumerate(value):
                    column_name = f"{topic}.{field}[{i}]"
                    row[column_name] = val
                    seen_columns.add(column_name)

        if len(row) > 1:
            rows.append(row)

    if rows:
        df = pd.DataFrame(rows)

        for col in seen_columns:
            if col not in df.columns:
                df[col] = np.nan

        df.sort_values("timestamp", inplace=True)
        df.ffill(inplace=True)
    
    return df


def read_can_txt_file(file_path: str):
    pattern = re.compile(
        r"\(([\d.]+)\)\s+can\d+\s+([0-9A-Fa-f]+)\s+\[\d+\]\s+((?:[0-9A-Fa-f]{2}\s+)+)"
    )

    can_conversions = pd.read_csv("/home/alvaro/log_plotter/can_conversions.csv", index_col=0)

    print(can_conversions)

    rows = []
    seen_columns = set()

    with open(file_path, "r", encoding="latin1") as f:
        for line in f:
            match = pattern.match(line.strip())
            if match:
                timestamp = float(match.group(1))
                can_id = str(hex(int(match.group(2), 16)))
                data_str = match.group(3).strip()
                data_bytes = [int(byte, 16) for byte in data_str.split()]

                if can_id in can_conversions.index:
                    bitIn = can_conversions["bitIn"][can_id]
                    bitFin = can_conversions["bitFin"][can_id]
                    value_bytes = [data_bytes[i] for i in range(bitIn, bitFin + 1) if i < len(data_bytes)]
                    if can_conversions["Signed"][can_id] == "False":
                        raw_int = int.from_bytes(value_bytes, byteorder="little", signed=False)
                    else:
                        raw_int = int.from_bytes(value_bytes, byteorder="little", signed=True)
                    row = {"timestamp": timestamp}
                    column_name = can_conversions["Name"][can_id] + " ("+can_id+")"
                    row[column_name] = raw_int*can_conversions["Scale"][can_id] + can_conversions["Offset"][can_id]
                    seen_columns.add(column_name)
                    if len(row) > 1:
                        rows.append(row)

                for i in range(1,4):
                    subid = can_id + str(i)

                    if subid in can_conversions.index:
                        bitIn = can_conversions["bitIn"][subid]
                        bitFin = can_conversions["bitFin"][subid]
                        value_bytes = [data_bytes[i] for i in range(bitIn, bitFin + 1) if i < len(data_bytes)]
                        if can_conversions["Signed"][subid] == "False":
                            raw_int = int.from_bytes(value_bytes, byteorder="little", signed=False)
                        else:
                            raw_int = int.from_bytes(value_bytes, byteorder="little", signed=True)
                        row = {"timestamp": timestamp}
                        column_name = can_conversions["Name"][subid] + " ("+can_id+")"
                        row[column_name] = raw_int*can_conversions["Scale"][subid] + can_conversions["Offset"][subid]
                        seen_columns.add(column_name)
                        if len(row) > 1:
                            rows.append(row)
                    
                
    if rows:
        df = pd.DataFrame(rows)

        for col in seen_columns:
            if col not in df.columns:
                df[col] = np.nan

        df.sort_values("timestamp", inplace=True)
        df.ffill(inplace=True)


    return df


def main():
    if len(sys.argv) > 1:
        file_path = sys.argv[1]
    else:
        root = tk.Tk()
        root.withdraw()  

        file_path = filedialog.askopenfilename(
            title="Select the log file",
            filetypes=[("MCAP files", "*.mcap"), ("txt files", "*.txt"), ("All files", "*.*")]
        )
        root.destroy()  

        if not file_path:
            messagebox.showerror("Error", "No file selected.")
            exit()

    df = pd.DataFrame()
    
    if file_path.endswith(".txt"):
        df = read_can_txt_file(file_path)
        # Convertir timestamp a datetime opcionalmente
        # df["timestamp"] = pd.to_datetime(df["timestamp"], unit='s')
    else:
        df = read_rosbag_mcap(file_path)  # tu función original para MCAP


    if not df.empty:
        plot_variables(df)
    else:
        print("No numeric data extracted for plotting.")


if __name__ == "__main__":
    main()