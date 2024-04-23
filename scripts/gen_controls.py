import os
import re
import json
from reportlab.lib.utils import ImageReader
from reportlab.pdfgen import canvas
from PIL import Image

file_path = os.path.split(os.path.realpath(__file__))[0] + '/'


def extract_comments(text):
    pattern = r'/\*RHRXBox\s*\n\s*\*Type:\s*(\d+)\s*\n\s*\*Btn:\s*(\d+)\s*\n\s*\*Desc:\s*(.*?)\*/'
    comments = re.findall(pattern, text, re.DOTALL)
    return [{'type': int(comment[0]), 'btn': int(comment[1].strip()), 'desc': comment[2].strip()} for comment in
            comments]


def open_file():
    java_file_path = file_path + '../src/main/java/frc/robot/Robot.java'
    with open(java_file_path, 'r') as file:
        java_code = file.read()

    comments = extract_comments(java_code)
    print(json.dumps(comments, indent=4))
    return comments


def get_text_location(btn):
    height = 753
    if btn == 1:
        return 783, height - 300
    if btn == 2:
        return 850, height - 230
    if btn == 3:
        return 700, height - 200
    if btn == 4:
        return 783, height - 150
    if btn == 5:
        return 201, 675
    if btn == 6:
        return 700, 675
    if btn == 7:
        return 400, height - 240
    if btn == 8:
        return 550, height - 240
    if btn == 9:
        return 24, height - 30
    if btn == 10:
        return 900, height - 30
    if btn == 21:
        return 320, height - 350
    if btn == 23:
        return 320, height - 450
    if btn == 31:
        return 181, height - 250
    if btn == 32:
        return 625, height - 400
    return 100, 100


def draw_text(c, btn, text):
    textobject = c.beginText(*get_text_location(btn))
    textobject.setFont("Helvetica", 30, leading=None)
    textobject.setFillColor((225, 0, 0))
    textobject.textLine(text)
    c.drawText(textobject)


def to_pdf(btns, type_id):
    controller_image = Image.open(file_path + "xbox" + str(type_id) + ".png")

    reportlab_pil_img = ImageReader(controller_image)

    # Create a PDF canvas
    c = canvas.Canvas(str(type_id) + "_buttons.pdf", pagesize=(1110, 753))
    c.drawImage(reportlab_pil_img, 0, 0, width=1110, height=753)

    for btn in btns:
        draw_text(c, btn['btn'], btn['desc'])

    c.save()


def main():
    cmds = open_file()
    cmds_driver = [cmd for cmd in cmds if cmd['type'] == 1]
    cmds_operator = [cmd for cmd in cmds if cmd['type'] == 2]
    to_pdf(cmds_driver, 1)
    to_pdf(cmds_operator, 2)


if __name__ == "__main__":
    main()
