#!/usr/bin/env python3
"""Editable A4 PowerPoint and matching vector PDF for the SO-101 exhibition."""
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
OUT=ROOT/'docs/exhibition';OUT.mkdir(parents=True,exist_ok=True)
W,H=595.2756,841.8898
BG='F6F5EF';NAVY='183C43';TEAL='008D82';MINT='DDF2EA';ORANGE='EFAB45';PALE='FFF0CC';GRAY='52676C';LINE='DEE5DF';WHITE='FFFFFF'
prs=Presentation();prs.slide_width=Pt(W);prs.slide_height=Pt(H)
slide=prs.slides.add_slide(prs.slide_layouts[6]);slide.background.fill.solid();slide.background.fill.fore_color.rgb=RGBColor.from_string(BG)
prs.core_properties.title='カメラで見つけて、ロボットがつかむ！'
prs.core_properties.subject='SO-101 exhibition introduction, A4 portrait'
prs.core_properties.author='robot_arm project'
pdf=canvas.Canvas(str(OUT/'robot_arm_exhibition_A4.pdf'),pagesize=(W,H))
pdf.setTitle('カメラで見つけて、ロボットがつかむ！')
pdf.setAuthor('robot_arm project')
pdfmetrics.registerFont(TTFont('JP','/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf'))
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
        parts=re.findall(r'[\x00-\x7f]+|[^\x00-\x7f]+',chunk)
        fonts=[('Helvetica-Bold' if bold else 'Helvetica') if ord(part[0])<128 else 'JP' for part in parts]
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

# Header
text(28,23,539,18,'SO-101  /  小さなロボット研究室',10,TEAL,True)
text(28,49,539,44,'カメラで見つけて、',32,NAVY,True)
text(28,92,539,45,'ロボットがつかむ！',33,TEAL,True)
text(29,141,539,21,'四角い目印をたよりに、ものを運びます。',13,GRAY)

# Main explanatory illustration
box(28,176,539,231,WHITE,LINE)
text(44,187,500,24,'２枚の目印で、「どこ？」がわかる',14,NAVY,True)
shape(MSO_SHAPE.RECTANGLE,44,353,507,22,'EDF2ED')
line(45,353,550,353,'CCDAD2',1)
camera(65,241,1.12)
line(118,264,253,338,TEAL,1.2,True)
line(118,264,386,327,TEAL,1.2,True)
arm(260,350,.91)
# object under fingers and destination
box(375,321,28,31,ORANGE)
marker(381,326,16,1)
marker(245,330,19,0)
circle(480,347,20,None,TEAL,1.5)
text(453,371,57,16,'運ぶ先',10,TEAL,False,'center')
arrow(407,310,470,310,ORANGE,2.5)
line(470,310,480,326,ORANGE,2)
box(44,293,142,43,MINT)
text(54,300,124,32,'目印 A\nロボットの場所',10.5,TEAL,True)
line(186,316,244,337,TEAL,1)
box(383,224,160,44,PALE)
text(393,231,141,32,'目印 B\n運ぶものの場所',10.5,NAVY,True)
line(400,268,391,321,ORANGE,1.2)
text(45,383,505,15,'右・左、近い・遠い、高い・低いを計算します。',11,GRAY)

# Three-step story
text(28,425,539,24,'どうやって運ぶの？',17,NAVY,True)
for i,(title,body) in enumerate([
 ('目印を見つける','カメラの画像から\nものの場所を調べます。'),
 ('道を考える','ぶつからないように\n動く道を計画します。'),
 ('つかんで運ぶ','つかんだものを\n決めた場所へ運びます。')]):
    x=28+i*184
    box(x,459,171,115,WHITE,LINE)
    circle(x+18,477,10,TEAL if i!=1 else 'C48120')
    text(x+12,469,12,18,str(i+1),11,WHITE,True,'center')
    text(x+34,467,132,22,title,12,NAVY,True)
    text(x+14,530,148,35,body,10.5,GRAY)
    if i==0:camera(x+68,494,.52)
    elif i==1:
        line(x+44,517,x+72,498,TEAL,2);line(x+72,498,x+107,515,TEAL,2);arrow(x+107,515,x+135,497,TEAL,2)
        for xx,yy in [(44,517),(72,498),(107,515)]:circle(x+xx,yy,3.5,ORANGE)
    else:
        line(x+67,496,x+67,516,NAVY,4);line(x+106,496,x+106,516,NAVY,4)
        line(x+67,496,x+106,496,NAVY,4);box(x+77,508,20,16,ORANGE)

# Architecture: child-readable captions, technical labels for interested visitors
text(28,591,539,25,'しくみをのぞいてみよう',17,NAVY,True)
box(28,623,539,126,WHITE,LINE)
box(40,636,91,77,MINT)
camera(67,646,.6)
text(43,690,85,19,'カメラ',11,NAVY,True,'center')
arrow(136,673,159,673,TEAL,2)
box(165,633,266,82,'EFF4F1')
text(175,637,246,20,'パソコン',11,NAVY,True,'center')
box(176,660,111,43,WHITE)
text(179,665,105,16,'目印の位置を計算',10,NAVY,True,'center')
text(179,683,105,16,'ArUco',10,TEAL,False,'center')
arrow(290,682,302,682,TEAL,1.5)
box(305,660,114,43,WHITE)
text(308,665,108,16,'動く道を計画',10,NAVY,True,'center')
text(308,683,108,16,'MoveIt 2',10,TEAL,False,'center')
arrow(437,673,459,673,TEAL,2)
box(465,636,90,77,MINT)
arm(494,681,.30)
text(468,690,84,19,'ロボット',11,NAVY,True,'center')
text(43,724,511,18,'ROS 2 が、情報と動きの合図をつなぎます。',10.5,GRAY,False,'center')

# Visitor question / footer
box(28,766,539,37,PALE)
text(40,775,515,23,'どこに目印を置くと、見つけてもらえるかな？',13,NAVY,True,'center')
text(28,816,539,17,'動くアームにはさわらず、スタッフといっしょに見てね。',10,GRAY,False,'center')

# Keep explanation separate from operating instructions; no false AI claims.
slide.notes_slide.notes_text_frame.text = ('展示用A4紹介ポスター。図は説明用の模式図。実装はArUcoによるマーカー検出・位置姿勢推定、MoveIt 2の軌道計画、ROS 2による連携。生成AIや物体認識AIで動いているという説明はしていません。図形と文章はPowerPointで編集可能。推奨フォント Noto Sans CJK JP（未導入環境では日本語フォントを指定）。印刷はA4縦・1ページ。')
prs.save(OUT/'robot_arm_exhibition_A4.pptx')
pdf.showPage();pdf.save()
(OUT/'poster_text.txt').write_text('\n\n'.join(texts)+'\n')
print(OUT/'robot_arm_exhibition_A4.pptx')
print(OUT/'robot_arm_exhibition_A4.pdf')
