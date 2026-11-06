#!/usr/bin/env python3
"""Formal A4 technical poster: manual teleoperation to visual pick-and-place."""
from pathlib import Path
import re
import sys, math, xml.etree.ElementTree as ET
sys.path.insert(0, '/tmp/so101_poster_deps')
from pptx import Presentation
from pptx.util import Pt
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE, MSO_CONNECTOR
from pptx.enum.text import MSO_ANCHOR
from pptx.oxml.xmlchemy import OxmlElement
from reportlab.pdfgen import canvas
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'docs/research';OUT.mkdir(parents=True,exist_ok=True)
W,H=595.2756,841.8898
BG='FFFFFF';NAVY='35272D';TEAL='82002D';MINT='F3E5EB';ORANGE='B78052';PALE='F8EEE4';GRAY='69565F';LINE='E4D7DD';WHITE='FFFFFF'
prs=Presentation();prs.slide_width=Pt(W);prs.slide_height=Pt(H)
slide=prs.slides.add_slide(prs.slide_layouts[6]);slide.background.fill.solid();slide.background.fill.fore_color.rgb=RGBColor.from_string(BG)
prs.core_properties.title='単眼カメラとArUcoマーカーを用いた小型ロボットアームの物体搬送'
prs.core_properties.subject='SO-101 exhibition introduction, A4 portrait'
prs.core_properties.author='robot_arm project'
pdf=canvas.Canvas(str(OUT/'robot_arm_research_A4.pdf'),pagesize=(W,H))
pdf.setTitle('単眼カメラとArUcoマーカーを用いた小型ロボットアームの物体搬送')
pdf.setAuthor('robot_arm project')
pdfmetrics.registerFont(TTFont('JP','/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf'))
pdfmetrics.registerFont(TTFont('Symbols','/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'))
texts=[]
def rgb(c):return tuple(int(c[i:i+2],16)/255 for i in (0,2,4))
pdf.setFillColorRGB(*rgb(BG));pdf.rect(0,0,W,H,fill=1,stroke=0)
def shape(kind,x,y,w,h,fill=WHITE,stroke=None,sw=1,r=10):
    s=slide.shapes.add_shape(kind,Pt(x),Pt(y),Pt(w),Pt(h))
    if fill:s.fill.solid();s.fill.fore_color.rgb=RGBColor.from_string(fill)
    else:s.fill.background()
    if stroke:s.line.color.rgb=RGBColor.from_string(stroke);s.line.width=Pt(sw)
    else:s.line.fill.background()
    pdf.setFillColorRGB(*rgb(fill or WHITE));pdf.setStrokeColorRGB(*rgb(stroke or WHITE));pdf.setLineWidth(sw)
    if kind==MSO_SHAPE.OVAL:pdf.ellipse(x,H-y-h,x+w,H-y,fill=bool(fill),stroke=bool(stroke))
    elif kind==MSO_SHAPE.ROUNDED_RECTANGLE:
        try:s.adjustments[0]=.12
        except Exception:pass
        pdf.roundRect(x,H-y-h,w,h,min(r,w/2,h/2),fill=bool(fill),stroke=bool(stroke))
    else:pdf.rect(x,H-y-h,w,h,fill=bool(fill),stroke=bool(stroke))
    return s

def box(x,y,w,h,fill=WHITE,stroke=None):return shape(MSO_SHAPE.ROUNDED_RECTANGLE,x,y,w,h,fill,stroke)
def circle(x,y,r,fill,stroke=None,sw=1):return shape(MSO_SHAPE.OVAL,x-r,y-r,r*2,r*2,fill,stroke,sw)
def line(x1,y1,x2,y2,color=NAVY,width=1.5,dashed=False):
    s=slide.shapes.add_connector(MSO_CONNECTOR.STRAIGHT,Pt(x1),Pt(y1),Pt(x2),Pt(y2));s.line.color.rgb=RGBColor.from_string(color);s.line.width=Pt(width)
    if dashed:
        from pptx.enum.dml import MSO_LINE_DASH_STYLE
        s.line.dash_style=MSO_LINE_DASH_STYLE.DASH
    pdf.setStrokeColorRGB(*rgb(color));pdf.setLineWidth(width);pdf.setLineCap(1)
    pdf.setDash(4,3) if dashed else pdf.setDash()
    pdf.line(x1,H-y1,x2,H-y2);pdf.setDash()

def arrow(x1,y1,x2,y2,color=TEAL,width=2):
    line(x1,y1,x2,y2,color,width)
    angle=math.atan2(y2-y1,x2-x1)
    for d in (-.52,.52):line(x2,y2,x2-7*math.cos(angle+d),y2-7*math.sin(angle+d),color,width)

def text(x,y,w,h,value,size=12,color=NAVY,bold=False,align='left'):
    value=value.replace("、","，").replace("。","．")
    texts.append(value)
    tb=slide.shapes.add_textbox(Pt(x),Pt(y),Pt(w),Pt(h));tf=tb.text_frame;tf.clear();tf.word_wrap=False
    tf.margin_left=tf.margin_right=tf.margin_top=tf.margin_bottom=0
    from pptx.enum.text import PP_ALIGN
    for i,chunk in enumerate(value.split('\n')):
        p=tf.paragraphs[0] if i==0 else tf.add_paragraph();p.alignment={'left':PP_ALIGN.LEFT,'center':PP_ALIGN.CENTER,'right':PP_ALIGN.RIGHT}[align]
        p.space_before=Pt(0);p.space_after=Pt(0);p.line_spacing=Pt(size*1.34)
        run=p.add_run();run.text=chunk;run.font.name='Noto Sans CJK JP';run.font.size=Pt(size);run.font.bold=bold;run.font.color.rgb=RGBColor.from_string(color)
        east=OxmlElement('a:ea');east.set('typeface','Noto Sans CJK JP');run._r.get_or_add_rPr().append(east)
        pdf.setFillColorRGB(*rgb(color))
        yy=H-y-size*1.05-i*size*1.34
        parts=re.findall(r'[\x00-\x7f]+|[→×·−≤≥±]+|[^\x00-\x7f→×·−≤≥±]+',chunk)
        fonts=[('Helvetica-Bold' if bold else 'Helvetica') if ord(part[0])<128 else ('Symbols' if part[0] in '→×·−≤≥±' else 'JP') for part in parts]
        widths=[pdfmetrics.stringWidth(part,font,size) for part,font in zip(parts,fonts)]
        xx=x+(w-sum(widths))/2 if align=='center' else (x+w-sum(widths) if align=='right' else x)
        for part,font,length in zip(parts,fonts,widths):
            pdf.setFont(font,size);pdf.drawString(xx,yy,part);xx+=length
    return tb

def camera(x,y,scale=1):
    box(x,y,45*scale,30*scale,TEAL)
    circle(x+24*scale,y+15*scale,10*scale,WHITE)
    circle(x+24*scale,y+15*scale,6*scale,NAVY)
    shape(MSO_SHAPE.RECTANGLE,x+7*scale,y-5*scale,14*scale,7*scale,TEAL)
    line(x+22*scale,y+30*scale,x+22*scale,y+43*scale,NAVY,2*scale)
    line(x+22*scale,y+43*scale,x+11*scale,y+50*scale,NAVY,2*scale)
    line(x+22*scale,y+43*scale,x+33*scale,y+50*scale,NAVY,2*scale)

def marker(x,y,size,idx):
    svg=ET.parse(ROOT/f'docs/aruco_markers/DICT_4X4_50_id{idx}_20mm.svg').getroot()
    shape(MSO_SHAPE.RECTANGLE,x-3,y-3,size+6,size+6,WHITE)
    for e in svg.findall('.//{http://www.w3.org/2000/svg}g/{http://www.w3.org/2000/svg}rect'):
        a,b,c,d=[float(e.get(k)) for k in ['x','y','width','height']]
        shape(MSO_SHAPE.RECTANGLE,x+(a-5)*size/20,y+(b-5)*size/20,c*size/20,d*size/20,'142B31')

def arm(x,y,s=1):
    # Schematic of the compact articulated SO-101: pale links, teal joints, two fingers.
    box(x-31*s,y-12*s,73*s,18*s,NAVY)
    points=[(x,y-17*s),(x+4*s,y-64*s),(x+57*s,y-115*s),(x+100*s,y-77*s)]
    for (a,b),(c,d) in zip(points,points[1:]):
        line(a,b,c,d,NAVY,22*s);line(a,b,c,d,'E8EEE9',17*s)
    for a,b in points:circle(a,b,10*s,TEAL,NAVY,1*s);circle(a,b,3*s,MINT)
    a,b=points[-1]
    line(a,b,a+12*s,b+20*s,NAVY,9*s)
    line(a+12*s,b+20*s,a+29*s,b+20*s,NAVY,6*s)
    line(a+8*s,b+22*s,a+7*s,b+39*s,NAVY,4*s)
    line(a+29*s,b+20*s,a+29*s,b+39*s,NAVY,4*s)

# Formal reading order: background -> architecture -> methods -> implementation -> conclusion.
text(26,18,350,13,'ROBOTICS  /  SYSTEM DEVELOPMENT',8,TEAL,True)
text(394,18,175,13,'演題番号：［記入欄］',8,GRAY,False,'right')
text(26,39,543,32,'コントローラー操作から',23,NAVY,True)
text(26,72,543,32,'視覚に基づく自動搬送へ',23,TEAL,True)
text(27,109,542,16,'単眼カメラ・ArUco・MoveIt 2によるロボットアームの統合制御',10.3,GRAY)
text(27,130,542,16,'発表者：［氏名］　所属：［所属名］',9.5,GRAY)
line(26,154,569,154,TEAL,2)

# 1 Background: development history is the main narrative, not an evaluation claim.
text(26,171,543,21,'1  背景・目的：人の操作から、視覚に基づく動作へ',12,NAVY,True)
box(26,201,245,77,'F4F0F2')
text(38,209,220,18,'2025年｜コントローラーで手動操作',10.3,NAVY,True)
text(38,232,180,35,'人が状況を判断し，\n入力に合わせてアームを動かす．',9,GRAY)
# Editable game controller illustration
box(229,241,31,20,TEAL)
line(235,247,235,255,WHITE,1.6);line(231,251,239,251,WHITE,1.6)
circle(251,247,1.5,WHITE);circle(255,251,1.5,WHITE)
circle(241,256,2.2,'E2B6C6');circle(248,256,2.2,'E2B6C6')
arrow(278,239,311,239,TEAL,2)
box(319,201,250,77,MINT)
text(331,209,226,18,'2026年｜カメラで位置を推定して搬送',10.3,TEAL,True)
text(331,232,170,35,'目印から物体の位置を推定．\n計画・把持・搬送を自動で実行．',9,GRAY)
arm(524,264,.25)
text(27,285,542,16,'目的：位置推定・動作計画・実行制御を統合し，手動操作への依存を減らす．',9.2,NAVY)

# 2 Architecture
text(26,315,543,20,'2  システム構成',12,NAVY,True)
for x,w,title,body in [
 (26,108,'画像取得','単眼USBカメラ\n画像＋校正情報'),
 (166,117,'位置・姿勢推定','ArUco / IPPE\n角点精密化・時間窓'),
 (316,117,'軌道計画','MoveIt 2 / OMPL\nIK・衝突判定'),
 (466,103,'実行・状態取得','ros2_control\nSO-101 / STS3215')]:
 box(x,344,w,57,MINT if x in (26,466) else 'F5F1F3')
 text(x+4,351,w-8,17,title,10,NAVY,True,'center')
 text(x+4,372,w-8,27,body,8.7,GRAY,False,'center')
camera(34,351,.28)
marker(174,351,10,1)
shape(MSO_SHAPE.RECTANGLE,323,352,14,10,None,TEAL,.8)
line(330,362,330,366,TEAL,.8);line(325,366,335,366,TEAL,.8)
for x1,x2 in [(139,161),(288,311),(438,461)]:arrow(x1,373,x2,373,TEAL,1.6)
text(27,409,541,16,'ROS 2 / TFで情報を共有。構成：5軸アーム＋グリッパー、単眼カメラ、ID0・ID1。',9,GRAY)

# 3 Perception / transforms
line(26,435,569,435,LINE,1)
text(26,448,543,21,'3  手法：2枚のマーカーから物体の相対位置を求める',12,NAVY,True)
for x,label in [(31,'台座 B'),(111,'カメラ C'),(191,'物体 O')]:
 box(x,482,65,29,MINT)
 text(x+2,489,61,17,label,9,NAVY,True,'center')
arrow(99,496,107,496,TEAL,1.2);arrow(179,496,187,496,TEAL,1.2)
marker(58,527,21,0);marker(208,527,21,1)
text(27,553,113,15,'ID0：台座に固定',8.9,GRAY,False,'center')
text(159,553,117,15,'ID1：物体に固定',8.9,GRAY,False,'center')
text(294,481,275,31,'2枚を同じ画像で観測し、取付オフセットを\n使って台座基準の物体原点を計算する。',9,GRAY)
box(291,519,278,48,'F4E9ED')
text(302,527,256,33,'台座を基準に、カメラと物体の位置関係を\nつないで、つかみに行く位置を求める。',10,NAVY)

# 4 Operational sequence, illustrated without error-correction details.
text(26,590,543,21,'4  実装：自動ピッキングと搬送シーケンス',12,NAVY,True)
for x,w,label in [(26,117,'開始指示・位置固定'),(166,117,'計画・移動'),(316,117,'把持・搬送・開放'),(466,103,'待機位置へ復帰')]:
 box(x,620,w,27,'F5F1F3');text(x+2,626,w-4,17,label,8.7,NAVY,False,'center')
for x1,x2 in [(147,162),(287,312),(437,462)]:arrow(x1,633,x2,633,TEAL,1.2)
# Pick-and-place illustration: lifted object, waypoint arc and destination.
line(38,733,282,733,LINE,1)
arm(69,726,.49)
box(131,700,17,20,ORANGE)
marker(135,704,9,1)
arrow(161,698,161,677,TEAL,1.5)
arrow(165,677,237,677,TEAL,1.5)
arrow(242,681,242,708,TEAL,1.5)
circle(242,722,12,None,TEAL,1.2)
text(38,738,247,14,'物体を把持し，上方を経由して搬送する．',8.4,GRAY,False,'center')
text(319,666,250,67,'・目印の位置から把持点を決定\n・衝突を確認し，軌道を計画\n・把持後は上方の経由点を通って搬送\n・開放後に待機位置へ戻り，次の指示を待つ',9.1,NAVY)

# 5 Conclusion: no preliminary-evaluation section, no unsupported claims.
box(26,757,543,46,'F4E9ED')
text(36,764,523,17,'5  今後の改良',10.5,NAVY,True)
text(36,782,523,17,'VLAを用いてより多様な物体を認識し行動計画を行う。',10.5,GRAY)
text(27,815,542,12,'技術参考：[1] OpenCV ArUco  [2] MoveIt 2　｜　レイアウト参考：[3] キンコーズ',7.4,GRAY)

slide.notes_slide.notes_text_frame.text='''A4縦・研究開発紹介ポスター。発表者・所属・演題番号は記入欄。
予備評価・比較グラフ・数値的な改善率は、利用者の指示により削除。
2025年の手動コントローラー操作はREADMEのPS4遠隔操作実装（2025-10-30）に基づく。2026年は本プロジェクトに実装したArUco・MoveIt・ROS 2の認識／計画／実行統合を記述。開始指示が必要で、無人で連続動作するとの主張はしていない。
利用者の指示により数式と誤差補正の説明は掲載せず，座標変換と搬送は図と文章で説明．第5節はVLAを今後導入する方針であり、現システムにVLAを実装済みという意味ではない。文字・構成図は編集可能。
座標変換：左上添字が基準、下添字が変換元。Cはカメラ光学座標。ID0・ID1は同一画像で観測。
ロゴや大学名は使用せず，臙脂色を基調とした独自のレイアウト．所属欄は記入欄．
[1] https://docs.opencv.org/4.13.0/d5/dae/tutorial_aruco_detection.html
[2] https://moveit.picknik.ai/main/doc/concepts/motion_planning.html
[3] https://www.kinkos.co.jp/column/print-of-society-of-poster-session-for-poster/
記事から情報量の絞り込み、番号付き見出し、左上から右下の読む順序、限定配色、余白の原則を参照。A0用文字サイズ等をそのまま適用せず、指定のA4縦に合わせた。特定学会の指定テンプレートではない。
'''
slide.notes_slide.notes_text_frame.text = slide.notes_slide.notes_text_frame.text.replace('、','，').replace('。','．')
prs.core_properties.title='コントローラー操作から視覚に基づく自動搬送へ'
prs.core_properties.subject='System development: teleoperation to visual pick-and-place, A4 portrait'
pdf.setTitle('コントローラー操作から視覚に基づく自動搬送へ')
prs.save(OUT/'robot_arm_research_A4.pptx')
pdf.showPage();pdf.save()
(OUT/'poster_text.txt').write_text('\n\n'.join(texts)+'\n')
print(OUT/'robot_arm_research_A4.pptx')
