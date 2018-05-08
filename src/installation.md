# Install virtual speech recognition

```shell
brew install pyaudio
sudo -H pip install pyaudio
sudo -H pip install SpeechRecognition
```

# Install virtual text to speech

```shell
sudo -H pip install gtts
sudo -H pip install playsound
```

If using macOS you will need to install `Objective-C` bindings with:

```shell
pip install pyobjc --user
```

# PyNaoqi in PyCharm
- Use version 2.5.5 from SoftBank download page
- Add `~/pynaoqi/lib/python2.7/site-packages` and `~/pynaoqi/lib` path to `.bash_profile`
- Open PyCharm and add `site-packages` to libraries (in interpreter of project options)
- Open `Edit configuration...` and add `lib` to `Enviroment variables`