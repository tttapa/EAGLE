"""
https://codereview.stackexchange.com/questions/162920/file-selection-button-for-jupyter-notebook
"""


import traitlets
from ipywidgets import widgets
from IPython.display import display
from tkinter import Tk, filedialog


class SelectFilesButton(widgets.Button):
    """A file widget that leverages tkinter.filedialog."""

    def __init__(self, **kwargs):
        super(SelectFilesButton, self).__init__()
        self.file = ""
        # Create the button.
        self.description = "Select log file"
        self.icon = "square-o"
        self.style.button_color = "orange"
        # Set on click behavior.
        self.on_click(self.select_files)
        self.kwargs = kwargs

    @staticmethod
    def select_files(b):
        """Generate instance of tkinter.filedialog.

        Parameters
        ----------
        b : obj:
            An instance of ipywidgets.widgets.Button 
        """
        # Create Tk root
        root = Tk()
        # Hide the main window
        root.withdraw()
        # Raise the root to the top of all windows.
        root.call('wm', 'attributes', '.', '-topmost', True)
        
        b.file = filedialog.askopenfilename(**b.kwargs)

        b.description = "Log Selected"
        b.icon = "check-square-o"
        b.style.button_color = "lightgreen"