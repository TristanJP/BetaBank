"""
Beta Bank Augmented Reality Climbing Solution
"""
import toga
from toga.style import Pack
from toga.style.pack import COLUMN, ROW
from main import Main
from capture import Capture


class BetaBank(toga.App):

    def startup(self):
        """
        Construct and show the Toga application.

        Usually, you would add your application to a main content box.
        We then create a main window (with a name matching the app), and
        show the main window.
        """

        main_box = toga.Box(style=Pack(direction=COLUMN))

        name_label = toga.Label(
            'File name to capture: ',
            style=Pack(padding=(0, 5))
        )
        self.name_input = toga.TextInput(style=Pack(flex=1))

        name_box = toga.Box(style=Pack(direction=ROW, padding=5))
        name_box.add(name_label)
        name_box.add(self.name_input)

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
            'File name to display: ',
            style=Pack(padding=(0, 5))
        )
        self.display_input = toga.TextInput(style=Pack(flex=1))

        dsiplay_box = toga.Box(style=Pack(direction=ROW, padding=5))
        dsiplay_box.add(display_label)
        dsiplay_box.add(self.display_input)

        button_run_webgl = toga.Button(
            'Run Webgl',
            on_press=self.run_webgl,
            style=Pack(padding=5)
        )
        main_box.add(dsiplay_box)
        main_box.add(button_run_webgl)

        self.main_window = toga.MainWindow(title=self.formal_name)
        self.main_window.content = main_box
        self.main_window.show()

    def capture_image(self, widget):
        cap = Capture()
        cap.take_pictures(self.name_input.value)

    def capture_video(self, widget):
        cap = Capture()
        cap.take_video(self.name_input.value)


    def run_webgl(self, widget):
        main = Main()
        main.start()
        if self.display_input.value is not "":
            main.run_webgl(display_path=f"test_videos_1920x1080/{self.display_input.value}")
        else:
            main.run_webgl()


def main():
    return BetaBank()
