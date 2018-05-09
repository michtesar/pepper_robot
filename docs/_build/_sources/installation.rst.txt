Installation
************

Install virtual speech recognition
==================================

- `brew install pyaudio`
- `sudo -H pip install pyaudio`
- `sudo -H pip install SpeechRecognition`

Install virtual text to speech
==============================


- `sudo -H pip install gtts`
- `sudo -H pip install playsound`

If using macOS you will need to install `Objective-C` bindings with:

- `pip install pyobjc --user`

PyNaoqi in PyCharm
==================
- Use version 2.5.5 from SoftBank download page
- Add `~/pynaoqi/lib/python2.7/site-packages` and `~/pynaoqi/lib` path to `.bash_profile`
- Open PyCharm and add `site-packages` to libraries (in interpreter of project options)
- Open `Edit configuration...` and add `lib` to `Enviroment variables`

Install VQA
===========
- `pip install tensorflow==1.0.0`
- Install TensorFlow Fold 0.0.1

Sharing a image of website to a Robot
=====================================
- Run in a folder you wont to host `python -m SimpleHTTPServer`
- Open a website at `<remote_computer_IP>:8000/<page_like_index.html>`

