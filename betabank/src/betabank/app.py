"""
Beta Bank Augmented Reality Climbing Solution
"""
import toga
from toga.style import Pack
from toga.style.pack import COLUMN, ROW
from .main import Main
from .capture import Capture


class BetaBank(toga.App):

    def startup(self):

        main_box = toga.Box(style=Pack(direction=COLUMN))

        capture_label = toga.Label(
            '\nCapture',
            style=Pack(padding=(0, 5))
        )
        main_box.add(capture_label)

        filename_label = toga.Label(
            'Captured Image/Video name: ',
            style=Pack(padding=(0, 5))
        )
        self.filename_input = toga.TextInput(style=Pack(flex=2))

        name_box = toga.Box(style=Pack(direction=ROW, padding=5))
        name_box.add(filename_label)
        name_box.add(self.filename_input)

        button_capture_image = toga.Button(
            'Capture Image',
            on_press=self.capture_image,
            style=Pack(padding=5)
        )
        main_box.add(name_box)
        main_box.add(button_capture_image)

        button_capture_video = toga.Button(
            'Capture Video',
            on_press=self.capture_video,
            style=Pack(padding=5)
        )
        main_box.add(button_capture_video)

        display_label = toga.Label(
            '\n\nDisplay',
            style=Pack(padding=(0, 5))
        )

        display_webgl_label = toga.Label(
            'Display Video name: ',
            style=Pack(padding=(0, 5))
        )
        self.display_input = toga.TextInput(style=Pack(flex=1))

        display_box = toga.Box(style=Pack(direction=ROW, padding=5))
        display_box.add(display_webgl_label)
        display_box.add(self.display_input)

        button_run_webgl = toga.Button(
            'Display in AR',
            on_press=self.run_webgl,
            style=Pack(padding=5)
        )
        main_box.add(display_label)
        main_box.add(display_box)
        main_box.add(button_run_webgl)

        self.main_window = toga.MainWindow(title=self.formal_name)
        self.main_window.content = main_box
        self.main_window.show()

    def capture_image(self, widget):
        cap = Capture()
        cap.take_pictures(self.filename_input.value)

    def capture_video(self, widget):
        cap = Capture()
        cap.take_video(self.filename_input.value)


    def run_webgl(self, widget):
        main = Main()
        main.start()
        if self.display_input.value is not "":
            main.run_webgl(display_path=f"test_videos_1920x1080/{self.display_input.value}")
        else:
            main.run_webgl()


def main():
    return BetaBank()
