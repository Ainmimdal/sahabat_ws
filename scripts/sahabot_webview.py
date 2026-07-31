#!/usr/bin/env python3
"""Native GPU-accelerated desktop window for the local SahaBot app."""

import sys

import gi

gi.require_version('Gtk', '3.0')
gi.require_version('Gdk', '3.0')
gi.require_version('WebKit2', '4.0')
from gi.repository import Gdk, Gtk, WebKit2  # noqa: E402


def main():
    uri = (
        sys.argv[1]
        if len(sys.argv) > 1
        else 'http://127.0.0.1:8000/?screen=explore'
    )

    settings = WebKit2.Settings()
    settings.set_enable_webgl(True)
    settings.set_enable_2d_canvas_acceleration(True)
    settings.set_enable_javascript(True)
    settings.set_media_playback_requires_user_gesture(False)
    settings.set_hardware_acceleration_policy(
        WebKit2.HardwareAccelerationPolicy.ALWAYS
    )

    webview = WebKit2.WebView()
    webview.set_settings(settings)

    window = Gtk.Window(title='SahaBot App')
    window.set_default_size(1280, 800)
    window.maximize()
    window.add(webview)
    window.connect('destroy', Gtk.main_quit)

    def key_pressed(_widget, event):
        if event.keyval == Gdk.KEY_F11:
            if window.get_window().get_state() & Gdk.WindowState.FULLSCREEN:
                window.unfullscreen()
            else:
                window.fullscreen()
            return True
        return False

    window.connect('key-press-event', key_pressed)
    window.show_all()
    webview.load_uri(uri)
    Gtk.main()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
