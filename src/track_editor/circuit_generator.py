#!/usr/bin/env python3
import sys, math, shutil
from pathlib import Path

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QSpinBox, QDoubleSpinBox, QScrollArea,
    QFrame, QProgressBar, QDialog, QFileDialog, QCheckBox,
    QGroupBox, QMessageBox, QStatusBar, QGridLayout, QComboBox,
    QLineEdit,
)
from PyQt5.QtCore  import Qt, QThread, pyqtSignal, QRectF, QPointF
from PyQt5.QtGui   import (
    QPainter, QPen, QBrush, QColor, QFont, QPainterPath, QCursor,
)
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from circuit_gen_core import (
    generate_circuit, export_pcd, export_json, export_both,
    new_session, _next_seed, _SHAPE_NAMES,
    _HAS_PROFILES, _HAS_MAPFILE,
)

#  Automatic detection of ARUSSim
_ARUSSIM_CANDIDATES = [
    "~/ros2_ws/src/ARUSSim/src/arussim/resources/tracks",
    "~/ros2_ws/install/arussim/share/arussim/resources/tracks",
]

def _find_arussim_tracks() -> Path | None:
    for c in _ARUSSIM_CANDIDATES:
        p = Path(c).expanduser()
        if p.exists(): return p
    return None

_ARUSSIM_TRACKS = _find_arussim_tracks()

# Track subcategories
TRACK_CATEGORIES = {
    "generated":   "Generated (auto)",
    "competition": "Competition",
    "test":        "Test",
    "custom":      "Custom",
}

C_BG       = QColor(235, 232, 225)
C_PANEL    = QColor(220, 216, 208)
C_CARD     = QColor(228, 225, 217)
C_TRACK    = QColor(200, 196, 188)
C_BORDER   = QColor(160, 155, 145)
C_BORDER_H = QColor(100,  95,  85)
C_TEXT     = QColor( 30,  28,  25)
C_TEXT2    = QColor( 90,  85,  75)
C_TEXT3    = QColor(145, 140, 130)
C_AMBER    = QColor(150,  95,  20)
C_GREEN    = QColor( 55, 110,  55)
C_RED      = QColor(140,  40,  40)
C_BLUE     = QColor( 50,  80, 160)
C_YELLOW   = QColor(160, 130,  20)
C_ORANGE   = QColor(170,  85,  20)

QSS = """
QMainWindow, QWidget, QDialog {
    background: #ebe8e1;
    color: #1e1c19;
    font-family: "Courier New", "DejaVu Sans Mono", monospace;
    font-size: 11px;
}
QScrollArea  { border: none; background: #ebe8e1; }
QScrollBar:vertical { background:#d8d4cc; width:10px; border:none; }
QScrollBar::handle:vertical { background:#a09890; min-height:20px; border:1px solid #888078; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height:0; }
QStatusBar { background:#d0ccc4; color:#706860; font-size:10px; border-top:1px solid #a09880; }
QGroupBox {
    font-size:9px; color:#908880; letter-spacing:2px;
    border:1px solid #b0a898; margin-top:14px; padding-top:12px;
    text-transform:uppercase; background:transparent;
}
QGroupBox::title { subcontrol-origin:margin; left:8px; padding:0 4px; background:#dcd8d0; }
QSpinBox, QDoubleSpinBox, QLineEdit {
    background:#f0ede6; border:1px solid #b0a898; color:#1e1c19;
    font-size:12px; padding:4px 6px;
}
QSpinBox:focus, QDoubleSpinBox:focus, QLineEdit:focus { border-color:#706050; }
QSpinBox::up-button, QSpinBox::down-button,
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
    background:#d8d4cc; border:none; width:14px;
}
QComboBox {
    background:#f0ede6; border:1px solid #b0a898; color:#1e1c19;
    font-size:11px; padding:3px 6px;
}
QComboBox:focus { border-color:#706050; }
QComboBox::drop-down { background:#d8d4cc; border:none; width:18px; }
QPushButton {
    background:#dcd8d0; border:1px solid #a09880; color:#302820;
    font-size:11px; letter-spacing:1px; padding:7px 14px;
}
QPushButton:hover    { background:#ccc8c0; border-color:#706050; }
QPushButton:pressed  { background:#bcb8b0; }
QPushButton:disabled { color:#b0a898; border-color:#c8c4bc; background:#d8d4cc; }
QPushButton#btn_gen {
    background:#e8e0d0; border:2px solid #967040; color:#604020;
    font-size:13px; font-weight:bold; letter-spacing:3px; padding:10px 20px;
}
QPushButton#btn_gen:hover    { background:#f0e8d8; border-color:#7a5030; }
QPushButton#btn_gen:disabled { background:#d8d4cc; border-color:#c0b8a8; color:#a09880; }
QPushButton#btn_arussim {
    background:#e0ead8; border:2px solid #507840; color:#304820;
    font-size:11px; font-weight:bold; letter-spacing:2px; padding:8px 14px;
}
QPushButton#btn_arussim:hover    { background:#d0e0c8; border-color:#305820; }
QPushButton#btn_arussim:disabled { background:#d8d4cc; border-color:#c0c8b8; color:#90a080; }
QProgressBar { background:#c8c4bc; border:1px solid #a09880; height:6px; }
QProgressBar::chunk { background:#967040; }
QCheckBox { color:#504840; font-size:10px; spacing:6px; }
QCheckBox::indicator { width:13px; height:13px; border:1px solid #a09880; background:#f0ede6; }
QCheckBox::indicator:checked { background:#967040; border-color:#604020; }
QLabel { color:#1e1c19; }
"""

#  GENERATOR THREAD
class GeneratorThread(QThread):
    circuit_ready = pyqtSignal(dict, int)
    progress      = pyqtSignal(int, int)
    finished_all  = pyqtSignal(int, int)

    def __init__(self, count, base_seed, params):
        super().__init__()
        self.count=count; self.base_seed=base_seed; self.params=params; self._stop=False

    def stop(self): self._stop=True

    def run(self):
        ok=0
        for i in range(self.count):
            if self._stop: break
            seed = _next_seed(self.base_seed, i)
            c = generate_circuit(seed,
                target_length=self.params.get('length'),
                track_width  =self.params.get('width'),
                cone_spacing =self.params.get('spacing'),
                force_shape  =self.params.get('shape'),
            )
            c['index']=i
            if c['compliant']: ok+=1
            self.circuit_ready.emit(c, i)
            self.progress.emit(i+1, self.count)
        self.finished_all.emit(ok, self.count)

#  CIRCUIT CARD
class CircuitCard(QWidget):
    clicked = pyqtSignal(dict)
    W, H, BAR = 256, 182, 28

    def __init__(self, circuit=None, parent=None):
        super().__init__(parent)
        self.circuit=circuit; self._hov=False
        self.setFixedSize(self.W, self.H)
        self.setCursor(QCursor(Qt.PointingHandCursor))

    def set_circuit(self, c): self.circuit=c; self.update()

    def _T(self, outer, inner, pad, W, H):
        pts = np.vstack([outer, inner])
        x0,x1 = pts[:,0].min(), pts[:,0].max()
        y0,y1 = pts[:,1].min(), pts[:,1].max()
        rx,ry = max(x1-x0,1), max(y1-y0,1)
        sc = min((W-2*pad)/rx, (H-2*pad)/ry)
        ox = pad+(W-2*pad-rx*sc)/2 - x0*sc
        oy = pad+(H-2*pad-ry*sc)/2 - y0*sc
        return lambda p: QPointF(p[0]*sc+ox, p[1]*sc+oy)

    def _poly(self, arr, T):
        path=QPainterPath(); pts=[T(p) for p in arr]
        path.moveTo(pts[0])
        for p in pts[1:]: path.lineTo(p)
        path.closeSubpath(); return path

    def paintEvent(self, _):
        p=QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        W,DH=self.W, self.H-self.BAR
        p.fillRect(0,0,self.W,self.H, C_CARD)
        pen=QPen(C_BORDER_H if self._hov else C_BORDER)
        pen.setWidth(1); p.setPen(pen); p.drawRect(0,0,self.W-1,self.H-1)

        if not self.circuit:
            p.setPen(C_TEXT3)
            p.drawText(QRectF(0,0,self.W,DH),Qt.AlignCenter,"generating...")
            p.end(); return

        c=self.circuit
        T=self._T(c['outer'],c['inner'],7,self.W,DH)

        tp=QPainterPath()
        tp.addPath(self._poly(c['outer'],T))
        tp.addPath(self._poly(c['inner'],T))
        tp.setFillRule(Qt.OddEvenFill)
        p.fillPath(tp, QBrush(C_TRACK))

        for arr,col,a in [(c['outer'],C_BLUE,110),(c['inner'],C_YELLOW,110)]:
            pen=QPen(QColor(col.red(),col.green(),col.blue(),a))
            pen.setWidthF(0.9); p.setPen(pen); p.drawPath(self._poly(arr,T))

        for col,cones,cr in [(C_BLUE,c['blue'],1.7),(C_YELLOW,c['yellow'],1.7)]:
            p.setPen(Qt.NoPen); p.setBrush(QBrush(col))
            for cone in cones: pt=T(cone); p.drawEllipse(pt,cr,cr)

        p.setPen(Qt.NoPen); p.setBrush(QBrush(C_ORANGE))
        p.drawEllipse(T(c['outer'][0]),4.5,4.5)

        ok=c['compliant']; by=DH
        p.fillRect(0,by,self.W,self.BAR, QColor(210,206,198))
        p.setPen(QPen(C_BORDER)); p.drawLine(0,by,self.W,by)

        fnt=QFont("Courier New"); fnt.setPixelSize(9); p.setFont(fnt)
        p.setPen(C_TEXT2)
        p.drawText(QRectF(6,by,80,self.BAR),Qt.AlignVCenter, c.get('shape','?'))
        p.drawText(QRectF(self.W//2-52,by,104,self.BAR),Qt.AlignVCenter|Qt.AlignHCenter,
                   f"{c['params']['length']:.0f}m  {c['params']['n_blue']+c['params']['n_yellow']} cones")
        p.setPen(C_GREEN if ok else C_RED)
        fnt2=QFont("Courier New"); fnt2.setPixelSize(9); fnt2.setBold(True); p.setFont(fnt2)
        p.drawText(QRectF(self.W-46,by,40,self.BAR),Qt.AlignVCenter|Qt.AlignRight,
                   "OK" if ok else "FAIL")

        if self._hov:
            pen=QPen(C_AMBER if ok else C_RED); pen.setWidth(2); p.setPen(pen)
            p.drawRect(1,1,self.W-2,self.H-2)
        p.end()

    def enterEvent(self,_): self._hov=True;  self.update()
    def leaveEvent(self,_): self._hov=False; self.update()
    def mousePressEvent(self,e):
        if e.button()==Qt.LeftButton and self.circuit:
            self.clicked.emit(self.circuit)

#  LARGE CANVAS (detail dialog)
class DetailCanvas(QWidget):
    def __init__(self, circuit, parent=None):
        super().__init__(parent)
        self.circuit=circuit; self.setMinimumSize(500,390)
        self.setStyleSheet("background:#ebe8e1;")

    def _T(self, outer, inner, pad):
        W,H=self.width(),self.height()
        pts=np.vstack([outer,inner])
        x0,x1=pts[:,0].min(),pts[:,0].max(); y0,y1=pts[:,1].min(),pts[:,1].max()
        rx,ry=max(x1-x0,1),max(y1-y0,1)
        sc=min((W-2*pad)/rx,(H-2*pad)/ry)
        ox=pad+(W-2*pad-rx*sc)/2-x0*sc; oy=pad+(H-2*pad-ry*sc)/2-y0*sc
        return lambda pt: QPointF(pt[0]*sc+ox, pt[1]*sc+oy)

    def _poly(self,arr,T):
        path=QPainterPath(); pts=[T(p) for p in arr]
        path.moveTo(pts[0])
        for p in pts[1:]: path.lineTo(p)
        path.closeSubpath(); return path

    def paintEvent(self,_):
        p=QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(0,0,self.width(),self.height(), C_BG)
        c=self.circuit; T=self._T(c['outer'],c['inner'],28)

        tp=QPainterPath(); tp.addPath(self._poly(c['outer'],T)); tp.addPath(self._poly(c['inner'],T))
        tp.setFillRule(Qt.OddEvenFill); p.fillPath(tp,QBrush(C_TRACK))

        cl=c['center']; n=len(cl); step=max(1,n//300)
        for i in range(0,n-step,step):
            p1=cl[i]; p2=cl[(i+step)%n]; pm=cl[(i-step)%n]
            ax=p2[0]-p1[0]; ay=p2[1]-p1[1]
            bx=p1[0]-pm[0]; by=p1[1]-pm[1]
            cross=abs(ax*by-ay*bx); den=math.hypot(ax,ay)*math.hypot(bx,by)
            k=cross/den if den>1e-6 else 0; t=min(k/0.45,1.0)
            r=int(50+t*100); g=int(80-t*60); b2=int(160-t*130)
            pen=QPen(QColor(r,g,b2,80)); pen.setWidthF(1.6); p.setPen(pen)
            p.drawLine(T(p1),T(p2))

        for arr,col,a in [(c['outer'],C_BLUE,170),(c['inner'],C_YELLOW,170)]:
            pen=QPen(QColor(col.red(),col.green(),col.blue(),a))
            pen.setWidthF(1.1); p.setPen(pen); p.drawPath(self._poly(arr,T))

        for col,cones,cr in [(C_BLUE,c['blue'],3.3),(C_YELLOW,c['yellow'],3.3)]:
            p.setBrush(QBrush(col)); p.setPen(Qt.NoPen)
            for cone in cones: p.drawEllipse(T(cone),cr,cr)

        pen=QPen(C_ORANGE); pen.setWidthF(2.5); pen.setStyle(Qt.DashLine); p.setPen(pen)
        p.drawLine(T(c['outer'][0]),T(c['inner'][0]))
        p.setBrush(QBrush(C_ORANGE)); p.setPen(Qt.NoPen)
        for bp in [c['outer'][0],c['inner'][0]]: p.drawEllipse(T(bp),6,6)

        cl2=c['center']; idx=len(cl2)//8
        if idx+40<len(cl2):
            a=T(cl2[idx]); b=T(cl2[idx+40])
            ang=math.atan2(b.y()-a.y(),b.x()-a.x())
            p.save(); p.translate(a); p.rotate(math.degrees(ang))
            p.setBrush(QBrush(QColor(30,25,20,70))); p.setPen(Qt.NoPen)
            p.drawPolygon([QPointF(20,0),QPointF(-9,6),QPointF(-9,-6)]); p.restore()
        p.end()

#  DETAIL DIALOG
class CircuitDetailDialog(QDialog):
    def __init__(self, circuit, parent=None):
        super().__init__(parent)
        self.circuit=circuit
        self.setWindowTitle(
            f"Circuit — seed {circuit['seed']}  [{circuit.get('shape','?')}]")
        self.setMinimumSize(980,640); self._build()

    def _sep(self):
        f=QFrame(); f.setFrameShape(QFrame.HLine)
        f.setStyleSheet("color:#b0a898;background:#b0a898;max-height:1px;"); return f

    def _build(self):
        root=QHBoxLayout(self); root.setSpacing(18); root.setContentsMargins(16,16,16,16)
        self.canvas=DetailCanvas(self.circuit,self); root.addWidget(self.canvas,3)

        panel=QWidget(); pl=QVBoxLayout(panel); pl.setSpacing(10)
        root.addWidget(panel,2)

        ok=self.circuit['compliant']
        lbl_id=QLabel(f"seed  {self.circuit['seed']}")
        lbl_id.setStyleSheet("font-size:17px;font-weight:bold;color:#1e1c19;letter-spacing:2px;")
        lbl_sh=QLabel(f"form:  {self.circuit.get('shape','?')}")
        lbl_sh.setStyleSheet("font-size:11px;color:#706050;letter-spacing:1px;")
        lbl_ok=QLabel("FSG 2026  COMPLIANT" if ok else "NOT COMPLIANT")
        lbl_ok.setStyleSheet(
            f"font-size:12px;font-weight:bold;letter-spacing:3px;"
            f"color:{'#377035' if ok else '#8c2828'};")
        pl.addWidget(lbl_id); pl.addWidget(lbl_sh); pl.addWidget(lbl_ok)
        pl.addWidget(self._sep())

        # Rules
        gb=QGroupBox("RULE CHECKING  (FS-Rules 2026)")
        gl=QGridLayout(gb); gl.setSpacing(4)
        for row,(key,r) in enumerate(self.circuit['rules'].items()):
            bg='#d8e8d0' if r['pass'] else '#e8d0d0'
            bdr='#90b890' if r['pass'] else '#c07878'
            w=QWidget(); w.setStyleSheet(f"background:{bg};border:1px solid {bdr};")
            wl=QHBoxLayout(w); wl.setContentsMargins(8,5,8,5)
            lk=QLabel(r['label']); lk.setStyleSheet("font-size:10px;color:#605850;min-width:130px;")
            lv=QLabel(r['fmt']);   lv.setStyleSheet(
                f"font-size:12px;font-weight:bold;color:{'#2a6028' if r['pass'] else '#882020'};")
            lr=QLabel(r['req']);   lr.setStyleSheet("font-size:9px;color:#908070;")
            lr.setAlignment(Qt.AlignRight|Qt.AlignVCenter)
            wl.addWidget(lk); wl.addWidget(lv,1); wl.addWidget(lr)
            gl.addWidget(w,row,0)
        pl.addWidget(gb)

        # Parameters
        gb2=QGroupBox("PARAMETERS"); pg=QGridLayout(gb2); pg.setSpacing(5)
        c=self.circuit
        stats=[("Length",      f"{c['params']['length']:.1f} m"),
               ("Track width", f"{c['params']['width']:.2f} m"),
               ("Cone spacing",    f"{c['params']['spacing']:.2f} m"),
               ("Blue cones",  str(c['params']['n_blue'])),
               ("Yellow cones",str(c['params']['n_yellow'])),
               ("Total cones",   str(c['params']['n_blue']+c['params']['n_yellow']))]
        for i,(k,v) in enumerate(stats):
            lk=QLabel(k); lk.setStyleSheet("font-size:9px;color:#908070;letter-spacing:1px;")
            lv=QLabel(v); lv.setStyleSheet("font-size:13px;font-weight:bold;color:#1e1c19;")
            pg.addWidget(lk,i,0); pg.addWidget(lv,i,1)
        pl.addWidget(gb2)

        # Export 
        gb3=QGroupBox("EXPORT FILE")
        el=QHBoxLayout(gb3)
        for txt,fn in [("PCD",     lambda: self._save('pcd')),
                       ("JSON",    lambda: self._save('json')),
                       ("PCD+JSON",lambda: self._save('both'))]:
            b=QPushButton(txt); b.clicked.connect(fn); el.addWidget(b)
        pl.addWidget(gb3)

        #  Export to ARUSSim
        gb4=QGroupBox("EXPORT TO ARUSSIM")
        al=QVBoxLayout(gb4); al.setSpacing(6)

        # Route
        path_row=QWidget(); prl=QHBoxLayout(path_row); prl.setContentsMargins(0,0,0,0)
        self.arussim_path_edit=QLineEdit()
        self.arussim_path_edit.setPlaceholderText("Route to tracks/ from ARUSSim...")
        if _ARUSSIM_TRACKS:
            self.arussim_path_edit.setText(str(_ARUSSIM_TRACKS))
        btn_browse=QPushButton("...")
        btn_browse.setFixedWidth(32)
        btn_browse.clicked.connect(self._browse_arussim)
        prl.addWidget(self.arussim_path_edit,1); prl.addWidget(btn_browse)
        al.addWidget(path_row)

        # Category
        cat_row=QWidget(); crl=QHBoxLayout(cat_row); crl.setContentsMargins(0,0,0,0)
        crl.addWidget(QLabel("Category:"))
        self.cat_combo=QComboBox()
        for key,label in TRACK_CATEGORIES.items():
            self.cat_combo.addItem(label, key)
        self.cat_combo.setCurrentIndex(0)  # generated by default
        crl.addWidget(self.cat_combo,1)
        al.addWidget(cat_row)

        btn_send=QPushButton("SEND TO ARUSSIM")
        btn_send.setObjectName("btn_arussim")
        btn_send.clicked.connect(self._send_to_arussim)
        al.addWidget(btn_send)
        pl.addWidget(gb4)

        leg=QLabel(
            "Blue = left border (outer)  "
            "Yellow = right border (inner)\n"
            "Orange = finish line  "
            "→ CW  |  Start (0,0) +X ARUSSim")
        leg.setStyleSheet("font-size:8px;color:#a09080;letter-spacing:1px;")
        leg.setWordWrap(True)
        pl.addWidget(leg); pl.addStretch()

    def _browse_arussim(self):
        folder=QFileDialog.getExistingDirectory(self,"Select tracks/ folder from ARUSSim")
        if folder: self.arussim_path_edit.setText(folder)

    def _send_to_arussim(self):
        tracks_root=Path(self.arussim_path_edit.text().strip())
        if not tracks_root.exists():
            QMessageBox.warning(self,"Invalid route",
                f"The folder does not exist:\n{tracks_root}")
            return
        cat=self.cat_combo.currentData()
        dest_dir=tracks_root/cat; dest_dir.mkdir(parents=True,exist_ok=True)
        seed=self.circuit['seed']; shape=self.circuit.get('shape','circuit')
        base=f"gen_{shape}_{seed}"
        try:
            export_pcd(self.circuit,  str(dest_dir/f"{base}.pcd"))
            export_json(self.circuit, str(dest_dir/f"{base}.json"))
            msg=(f"Circuit sent to ARUSSim:\n"
                 f"  {cat}/{base}.pcd\n  {cat}/{base}.json\n\n"
                 f"Remember to run colcon build and copy\n"
                 f"the files to install/ if necessary.")
            QMessageBox.information(self,"Sent to ARUSSim", msg)
            if self.parent():
                self.parent().statusBar().showMessage(
                    f"ARUSSim: {cat}/{base}.pcd+.json")
        except Exception as e:
            QMessageBox.critical(self,"Export error",str(e))

    def _save(self, fmt):
        seed=self.circuit['seed']; base=f"circuit_{seed}"
        if fmt=='pcd':
            p,_=QFileDialog.getSaveFileName(self,"Save PCD",base+".pcd","PCD (*.pcd)")
            if p: export_pcd(self.circuit,p)
        elif fmt=='json':
            p,_=QFileDialog.getSaveFileName(self,"Save JSON",base+".json","JSON (*.json)")
            if p: export_json(self.circuit,p)
        else:
            p,_=QFileDialog.getSaveFileName(self,"Save circuit",base,"All (*)")
            if p: export_both(self.circuit,str(p))


TRACK_CATEGORIES = {
    "generated":   "Generated (auto)",
    "competition": "Competition",
    "test":        "Test",
    "custom":      "Custom",
}

#  MAIN WINDOW
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Circuit Generator  FSG 2026")
        self.setMinimumSize(1200,760)
        self.setStyleSheet(QSS)
        self._circuits=[]; self._cards=[]; self._thread=None
        self._arussim_tracks=_ARUSSIM_TRACKS
        self._build()
        arussim_status=f"ARUSSim: {self._arussim_tracks}" if self._arussim_tracks else "ARUSSim: not detected"
        self.statusBar().showMessage(
            f"Ready  |  shapes={len(_SHAPE_NAMES)}  |  {arussim_status}")

    def _build(self):
        root=QWidget(); self.setCentralWidget(root)
        rl=QHBoxLayout(root); rl.setSpacing(0); rl.setContentsMargins(0,0,0,0)
        rl.addWidget(self._build_panel(),0)
        rl.addWidget(self._build_main(), 1)
        self.setStatusBar(QStatusBar())

    def _build_panel(self):
        panel=QWidget(); panel.setFixedWidth(294)
        panel.setStyleSheet("background:#dcd8d0;border-right:1px solid #b0a898;")
        ll=QVBoxLayout(panel); ll.setSpacing(10); ll.setContentsMargins(14,14,14,14)

        title=QLabel("CIRCUIT GENERATOR")
        title.setStyleSheet(
            "font-size:15px;font-weight:bold;color:#604020;"
            "letter-spacing:4px;line-height:1.5;")
        sub=QLabel("FS-Rules 2026 v1.1  /  DS 1")
        sub.setStyleSheet("font-size:9px;color:#908070;letter-spacing:2px;")
        ll.addWidget(title); ll.addWidget(sub)
        ll.addWidget(self._sep())

        # Generation
        gb=self._group("GENERATION"); gl=QVBoxLayout(gb); gl.setSpacing(6)
        self.sp_count=self._spin("Circuits:",    1,500,12,gl)
        self.sp_seed =self._spin("Base seed:", 0,999999,2026,gl)
        shape_row=QWidget(); sl=QHBoxLayout(shape_row); sl.setContentsMargins(0,0,0,0)
        sl.addWidget(QLabel("Shape:"))
        self.cb_shape=QComboBox()
        self.cb_shape.addItem("random",None)
        for sn in _SHAPE_NAMES: self.cb_shape.addItem(sn,sn)
        sl.addWidget(self.cb_shape,1); gl.addWidget(shape_row)
        self.chk_ok=QCheckBox("Only those compliant with FSG 2026")
        self.chk_ok.setChecked(True); gl.addWidget(self.chk_ok)
        ll.addWidget(gb)

        # Track
        gb2=self._group("TRACK  (0 = free)"); tl=QVBoxLayout(gb2); tl.setSpacing(6)
        self.ds_length =self._dspin("Length (m):",  0,500,0,tl)
        self.ds_width  =self._dspin("Width (m):",   0,  8,0,tl)
        self.ds_spacing=self._dspin("Spacing (m):",0,  5,0,tl)
        ll.addWidget(gb2)

        # Main button
        self.btn_gen=QPushButton("GENERATE")
        self.btn_gen.setObjectName("btn_gen")
        self.btn_gen.clicked.connect(self._on_gen); ll.addWidget(self.btn_gen)

        self.pbar=QProgressBar(); self.pbar.setTextVisible(False); self.pbar.setFixedHeight(6)
        ll.addWidget(self.pbar)

        self.lbl_stats=QLabel("No data")
        self.lbl_stats.setStyleSheet("font-size:9px;color:#908070;letter-spacing:1px;")
        self.lbl_stats.setWordWrap(True); ll.addWidget(self.lbl_stats)
        ll.addWidget(self._sep())

        #  Export file 
        gb3=self._group("EXPORT FILE"); el=QVBoxLayout(gb3); el.setSpacing(5)
        self.btn_epcd =QPushButton("Export all  —  PCD")
        self.btn_ejson=QPushButton("Export all  —  JSON")
        self.btn_eboth=QPushButton("Export all  —  PCD + JSON")
        for b in [self.btn_epcd,self.btn_ejson,self.btn_eboth]:
            b.setEnabled(False); b.clicked.connect(self._on_export_all); el.addWidget(b)
        ll.addWidget(gb3)

        #  Send to ARUSSim
        gb4=self._group("SEND TO ARUSSIM"); al=QVBoxLayout(gb4); al.setSpacing(5)

        # Track route
        path_row=QWidget(); prl=QHBoxLayout(path_row); prl.setContentsMargins(0,0,0,0)
        self.arussim_path=QLineEdit()
        self.arussim_path.setPlaceholderText("Route to tracks/...")
        if self._arussim_tracks:
            self.arussim_path.setText(str(self._arussim_tracks))
        self.arussim_path.setStyleSheet("font-size:9px;padding:3px 5px;")
        btn_b=QPushButton("..."); btn_b.setFixedWidth(28)
        btn_b.clicked.connect(self._browse_arussim)
        prl.addWidget(self.arussim_path,1); prl.addWidget(btn_b)
        al.addWidget(path_row)

        # Destination category
        cat_row=QWidget(); crl=QHBoxLayout(cat_row); crl.setContentsMargins(0,0,0,0)
        crl.addWidget(QLabel("Category:"))
        self.cat_combo=QComboBox()
        for key,label in TRACK_CATEGORIES.items():
            self.cat_combo.addItem(label, key)
        crl.addWidget(self.cat_combo,1); al.addWidget(cat_row)

        self.btn_arussim=QPushButton("SEND ALL TO ARUSSIM")
        self.btn_arussim.setObjectName("btn_arussim")
        self.btn_arussim.setEnabled(False)
        self.btn_arussim.clicked.connect(self._on_send_arussim)
        al.addWidget(self.btn_arussim)
        ll.addWidget(gb4)

        self.btn_clear=QPushButton("Clear grid")
        self.btn_clear.setEnabled(False)
        self.btn_clear.clicked.connect(self._clear)
        ll.addWidget(self.btn_clear)
        ll.addStretch()

        leg=QLabel(
            "Blue = left border\n"
            "Yellow = right border\n"
            "Orange = exit / finish\n"
            "→  CW  |  Start (0,0) +X ARUSSim")
        leg.setStyleSheet("font-size:8px;color:#b0a090;letter-spacing:1px;")
        ll.addWidget(leg)
        return panel

    def _build_main(self):
        w=QWidget(); wl=QVBoxLayout(w); wl.setSpacing(0); wl.setContentsMargins(0,0,0,0)
        bar=QWidget(); bar.setStyleSheet("background:#d0ccc4;border-bottom:1px solid #b0a898;")
        bl=QHBoxLayout(bar); bl.setContentsMargins(14,8,14,8)
        self.lbl_bar=QLabel("Configure the parameters and click  GENERATE")
        self.lbl_bar.setStyleSheet("font-size:10px;color:#908070;letter-spacing:2px;")
        bl.addWidget(self.lbl_bar); bl.addStretch()
        wl.addWidget(bar)

        self.prog_top=QProgressBar(); self.prog_top.setTextVisible(False)
        self.prog_top.setFixedHeight(3)
        self.prog_top.setStyleSheet(
            "QProgressBar{background:#c8c4bc;border:none;}"
            "QProgressBar::chunk{background:#967040;}")
        wl.addWidget(self.prog_top)

        self.scroll=QScrollArea(); self.scroll.setWidgetResizable(True)
        self.grid_w=QWidget(); self.grid_w.setStyleSheet("background:#ebe8e1;")
        self.grid_l=QGridLayout(self.grid_w)
        self.grid_l.setSpacing(8); self.grid_l.setContentsMargins(12,12,12,12)
        self.grid_l.setAlignment(Qt.AlignTop|Qt.AlignLeft)
        self.scroll.setWidget(self.grid_w); wl.addWidget(self.scroll)
        return w

    def _sep(self):
        f=QFrame(); f.setFrameShape(QFrame.HLine)
        f.setStyleSheet("color:#b0a898;background:#b0a898;max-height:1px;"); return f

    def _group(self, title):
        gb=QGroupBox(title)
        gb.setStyleSheet(
            "QGroupBox{font-size:9px;color:#908070;letter-spacing:2px;"
            "border:1px solid #b0a898;margin-top:14px;padding-top:12px;"
            "background:transparent;}"
            "QGroupBox::title{subcontrol-origin:margin;left:8px;"
            "padding:0 4px;background:#dcd8d0;}")
        return gb

    def _spin(self,lbl,mn,mx,val,pl):
        w=QWidget(); l=QHBoxLayout(w); l.setContentsMargins(0,0,0,0); l.setSpacing(6)
        lb=QLabel(lbl); lb.setStyleSheet("font-size:10px;color:#706050;")
        sp=QSpinBox(); sp.setRange(mn,mx); sp.setValue(val)
        l.addWidget(lb,1); l.addWidget(sp); pl.addWidget(w); return sp

    def _dspin(self,lbl,mn,mx,val,pl):
        w=QWidget(); l=QHBoxLayout(w); l.setContentsMargins(0,0,0,0); l.setSpacing(6)
        lb=QLabel(lbl); lb.setStyleSheet("font-size:10px;color:#706050;")
        sp=QDoubleSpinBox(); sp.setRange(mn,mx); sp.setValue(val); sp.setDecimals(1)
        l.addWidget(lb,1); l.addWidget(sp); pl.addWidget(w); return sp

    def _browse_arussim(self):
        folder=QFileDialog.getExistingDirectory(self,"Select ARUSsim tracks/ folder")
        if folder: self.arussim_path.setText(folder)

    #  Logic of generation 
    def _on_gen(self):
        if self._thread and self._thread.isRunning():
            self._thread.stop(); self._reset_btn(); return
        self._clear(); new_session()

        count=self.sp_count.value(); seed=self.sp_seed.value()
        params={}
        if self.ds_length.value()>0:  params['length'] =self.ds_length.value()
        if self.ds_width.value()>0:   params['width']  =self.ds_width.value()
        if self.ds_spacing.value()>0: params['spacing']=self.ds_spacing.value()
        sv=self.cb_shape.currentData()
        if sv: params['shape']=sv

        self.btn_gen.setText("STOP"); self.btn_gen.setEnabled(True)
        self.pbar.setRange(0,count); self.pbar.setValue(0)
        self.prog_top.setRange(0,count); self.prog_top.setValue(0)
        self.btn_clear.setEnabled(False)
        for b in [self.btn_epcd,self.btn_ejson,self.btn_eboth,self.btn_arussim]:
            b.setEnabled(False)

        self._thread=GeneratorThread(count,seed,params)
        self._thread.circuit_ready.connect(self._on_ready)
        self._thread.progress.connect(self._on_progress)
        self._thread.finished_all.connect(self._on_done)
        self._thread.start()

    def _on_ready(self, circuit, _):
        if self.chk_ok.isChecked() and not circuit['compliant']: return
        self._circuits.append(circuit)
        card=CircuitCard(circuit); card.clicked.connect(self._show_detail)
        self._cards.append(card)
        n=len(self._cards)-1; nc=self._ncols()
        self.grid_l.addWidget(card,n//nc,n%nc)

    def _on_progress(self, done, total):
        self.pbar.setValue(done); self.prog_top.setValue(done)
        ok=sum(1 for c in self._circuits if c['compliant'])
        self.lbl_stats.setText(
            f"Processed: {done}/{total}\n"
            f"Displayed:  {len(self._circuits)}\n"
            f"Compliant:  {ok}")
        self.lbl_bar.setText(f"Generating...  {done} / {total}")

    def _on_done(self, ok, total):
        self._reset_btn(); self.btn_clear.setEnabled(True)
        has=bool(self._circuits)
        for b in [self.btn_epcd,self.btn_ejson,self.btn_eboth]: b.setEnabled(has)
        self.btn_arussim.setEnabled(has)
        rate=ok/total*100 if total else 0
        self.lbl_stats.setText(
            f"Total: {total}  —  Compliant: {ok}  ({rate:.0f}%)\n"
            f"Displayed: {len(self._circuits)}")
        self.lbl_bar.setText(
            f"{len(self._circuits)} circuits  —  {ok}/{total} compliant  ({rate:.0f}%)")
        self.statusBar().showMessage(f"Done — {ok}/{total} compliant FSG 2026")
    def _reset_btn(self):
        self.btn_gen.setText("GENERATE"); self.btn_gen.setEnabled(True)

    def _show_detail(self, circuit):
        dlg=CircuitDetailDialog(circuit,self)
        dlg.setStyleSheet(QSS); dlg.exec_()

    def _clear(self):
        for c in self._cards: self.grid_l.removeWidget(c); c.deleteLater()
        self._cards=[]; self._circuits=[]
        self.pbar.setValue(0); self.prog_top.setValue(0)
        self.lbl_stats.setText("No data")
        self.lbl_bar.setText("Configure the parameters and click  GENERATE")
        self.btn_clear.setEnabled(False)
        for b in [self.btn_epcd,self.btn_ejson,self.btn_eboth,self.btn_arussim]:
            b.setEnabled(False)

    def _on_export_all(self):
        folder=QFileDialog.getExistingDirectory(self,"Export folder")
        if not folder: return
        txt=self.sender().text()
        if "JSON" in txt and "PCD" not in txt: fmt='json'
        elif "PCD" in txt and "JSON" not in txt: fmt='pcd'
        else: fmt='both'
        out=Path(folder)
        for c in self._circuits:
            base=str(out/f"circuit_{c['seed']}_{c.get('shape','?')}")
            if fmt=='pcd':   export_pcd(c,base+'.pcd')
            elif fmt=='json':export_json(c,base+'.json')
            else:            export_both(c,base)
        QMessageBox.information(
            self,"Full export",
            f"{len(self._circuits)} circuits → {folder}")

    def _on_send_arussim(self):
        """Send all the displayed tracks to ARUSSim/tracks/<category>/"""
        tracks_root=Path(self.arussim_path.text().strip())
        if not tracks_root.exists():
            QMessageBox.warning(self,"Invalid path",
                f"The folder does not exist:\n{tracks_root}\n\n"
                f"Configure the path to the tracks/ folder of ARUSSim")
            return

        cat=self.cat_combo.currentData()
        dest_dir=tracks_root/cat; dest_dir.mkdir(parents=True,exist_ok=True)

        ok=0; errors=[]
        for c in self._circuits:
            seed=c['seed']; shape=c.get('shape','circuit')
            base=f"gen_{shape}_{seed}"
            try:
                export_pcd(c,  str(dest_dir/f"{base}.pcd"))
                export_json(c, str(dest_dir/f"{base}.json"))
                ok+=1
            except Exception as e:
                errors.append(f"{base}: {e}")

        msg=(f"{ok}/{len(self._circuits)} circuits sent to:\n"
             f"  {tracks_root}/{cat}/\n\n"
             f"To use them in ARUSSim:\n"
             f"  1. colcon build --packages-select arussim\n"
             f"  2. Select '{cat}/<name>' in the simulator dropdown")
        if errors:
            msg+=f"\n\nErrors ({len(errors)}):\n"+"\n".join(errors[:5])
        QMessageBox.information(self,"Sent to ARUSSim", msg)
        self.statusBar().showMessage(f"ARUSSim: {ok} circuits → {cat}/")

    def _ncols(self):
        vp=self.scroll.viewport()
        w=vp.width() if vp else 900
        return max(1,(w-24)//(CircuitCard.W+8))

    def resizeEvent(self,e):
        super().resizeEvent(e)
        if self._cards:
            nc=self._ncols()
            for i,c in enumerate(self._cards): self.grid_l.addWidget(c,i//nc,i%nc)

#  MAIN
if __name__ == '__main__':
    app=QApplication(sys.argv)
    app.setApplicationName("Circuit Generator")
    win=MainWindow(); win.show()
    sys.exit(app.exec_())
