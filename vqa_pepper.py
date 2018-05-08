"""
PEPPER ROBOT - Implementation of N2NMN model (https://github.com/ronghanghu/n2nmn)

This code implements the visual question answering model from (R. Hu, J. Andreas et al., 2017) into Pepper robot. The model was trained on CLEVR dataset and is thus able to recognize object primitives, their attributes and spaital relations.

Date: 4.5.2018
Author: Gabriela Sejnova <gabriela.sejnova@cvut.cz>
Copyright (c) CIIRC CTU in Prague  - All Rights Reserved
"""

from __future__ import absolute_import, division, print_function

import argparse, sys

parser = argparse.ArgumentParser()
parser.add_argument('--gpu_id', type=int, default=0)
args = parser.parse_args()
gpu_id = args.gpu_id  # set GPU id to use
import os;

os.environ["CUDA_VISIBLE_DEVICES"] = str(gpu_id)
from robot import Pepper

sys.path.append('./exp_clevr/data/')
sys.path.append('./')
import numpy as np
from numpy import array
import text_processing
import tensorflow as tf
from nmn3_assembler import Assembler
from nmn3_model import NMN3Model
from models_clevr import vgg_net
from glob import glob
import skimage.io, skimage.color
import qi
import speech_recognition as sr
import time, paramiko, os, cv2

IP = "pepper.local"
username = "nao"
password = "nao"
PORT = 9559

""" Parameters """
# Image
H = 320
W = 480
H_feat = 10
W_feat = 15
D_feat = 512

# NN module parameters
embed_dim_txt = 300
embed_dim_nmn = 300
lstm_dim = 512
num_layers = 2
T_encoder = 45
T_decoder = 20
prune_filter_module = True


class PepperDescribeScn(object):

    def __init__(self, ip_address, port=9559):
        self.session = qi.Session()
        self.session.connect("tcp://" + IP + ":" + str(PORT))

        self.awareness = self.session.service("ALBasicAwareness")
        self.motion_service = self.session.service("ALMotion")
        self.photo_service = self.session.service("ALPhotoCapture")
        self.tts = self.session.service("ALTextToSpeech")
        self.recorder = self.session.service("ALAudioRecorder")
        self.pepper = Pepper(IP)

        """Pepper looks at table, takes image, listens to question and says answer """
        self.look_at_table()
        self.scene_img = self.get_camera_img()
        while True:
            try:
                self.question = self.record_question()

            except:
                self.tts.say("Sorry, i didn't hear you. can you repeat it?")
                continue
            break

        vqa = VQA(self.question, self.scene_img)
        vqa.feature_extraction()
        vqa.load_nmn3_model()
        self.pepper_guess = vqa.run_test()
        self.tts.say(self.pepper_guess)
        time.sleep(1)
        self.tts.say("Press enter to start over.")
        raw_input("Press Enter to continue...")

    def look_at_table(self):
        self.awareness.pauseAwareness()
        self.motion_service.angleInterpolationWithSpeed(["HeadYaw", "HeadPitch"], [-0.1, 0.4], 0.2)
        self.motion_service.setStiffnesses(["HeadYaw", "HeadPitch"], [0.0, 0.0])

    def get_camera_img(self):
        """take image from top camera"""
        self.img = self.pepper.get_camera_frame()

        """ Resize and crop image to fit desired resolution """
        self.resized_img = cv2.resize(self.img, (480, 320))
        self.img_file = self.img[20:340, 0:480]
        cv2.imshow("Seen image", self.img_file)
        cv2.waitKey(2)
        self.awareness.resumeAwareness()
        scene_img = self.img_file

        return scene_img

    def record_question(self):
        """record question, save to Pepper """
        self.tts.say("i can see the scene")
        self.tts.say("what is your question?")
        self.filename = "/home/nao/speech.wav"
        self.recorder.stopMicrophonesRecording()
        self.recorder.startMicrophonesRecording(self.filename, "wav", 48000, (0, 0, 1, 0))
        time.sleep(4)
        self.recorder.stopMicrophonesRecording()

        """ SSH connection and transfer """
        paramiko.util.log_to_file('/tmp/paramiko.log')
        paramiko.util.load_host_keys(os.path.expanduser('~/.ssh/known_hosts'))
        self.remote_wav_path = '/home/nao/speech.wav'
        self.local_wav_path = './exp_clevr/data/sound/speech.wav'
        self.ssh = paramiko.SSHClient()
        self.ssh.set_missing_host_key_policy(
            paramiko.AutoAddPolicy())
        self.ssh.connect(IP, username=username, password=password)
        self.sftp = self.ssh.open_sftp()
        self.sftp.get(self.remote_wav_path, self.local_wav_path)
        self.sftp.close()
        self.ssh.close()
        self.speech = sr.AudioFile(self.local_wav_path)

        """ Recognize record with Google API """
        self.r = sr.Recognizer()
        with self.speech as source:
            audio = self.r.record(source)
            self.recognized = self.r.recognize_google(audio, language="en-US")
            self.recognized = self.recognized.replace("wall", "ball")
            self.recognized = self.recognized.replace("Bull", "ball")
            self.recognized = self.recognized.replace("mall", "ball")
            self.recognized = self.recognized.replace("moon", "ball")
            self.recognized = self.recognized.replace("colour", "color")
            question = self.recognized
        print("Recognized:", question)

        return question


class VQA(object):

    def __init__(self, question, scene_img):
        """ Data files """
        self.question = question
        self.scene_img = scene_img
        self.snapshot_file = './exp_clevr/tfmodel/clevr_rl_gt_layout/00050000'
        self.vocab_question_file = './exp_clevr/data/vocabulary_clevr.txt'
        self.vocab_layout_file = './exp_clevr/data/vocabulary_layout.txt'
        self.vocab_answer_file = './exp_clevr/data/answers_clevr.txt'
        self.vgg_net_model = './exp_clevr/tfmodel/vgg_net/vgg_net.tfmodel'

    def feature_extraction(self):
        """ Extracts visual features from image via VGG neural network """
        self.image_batch = tf.placeholder(tf.float32, [1, H, W, 3])
        self.pool5 = vgg_net.vgg_pool5(self.image_batch, 'vgg_net')
        self.saver = tf.train.Saver()
        self.sess = tf.Session(config=tf.ConfigProto(gpu_options=tf.GPUOptions(allow_growth=True)))
        self.saver.restore(self.sess, self.vgg_net_model)
        # im = skimage.io.imread(image_basedir)[..., :3]
        # im = skimage.io.scene_img[..., :3]
        self.im_val = (self.scene_img[np.newaxis, ...] - vgg_net.channel_mean)
        self.pool5_val = self.pool5.eval({self.image_batch: self.im_val}, self.sess)
        self.sess.close()
        tf.reset_default_graph()

    def load_nmn3_model(self):
        """Initialize model session"""
        self.sess = tf.Session(config=tf.ConfigProto(
            gpu_options=tf.GPUOptions(allow_growth=True),
            allow_soft_placement=False, log_device_placement=False))

        self.raw_question = self.question
        self.question_tokens = text_processing.tokenize(self.raw_question)
        self.assembler = Assembler(self.vocab_layout_file)
        self.vocab_dict = text_processing.VocabDict(self.vocab_question_file)
        self.answer_dict = text_processing.VocabDict(self.vocab_answer_file)

        self.num_vocab_txt = self.vocab_dict.num_vocab
        self.num_vocab_nmn = len(self.assembler.module_names)
        self.num_choices = self.answer_dict.num_vocab

        """Network inputs - placeholders"""
        self.input_seq_batch = tf.placeholder(tf.int32, [None, None])
        self.seq_length_batch = tf.placeholder(tf.int32, [None])
        self.image_feat_batch = tf.placeholder(tf.float32, [None, H_feat, W_feat, D_feat])
        self.expr_validity_batch = tf.placeholder(tf.bool, [None])

        """The model for testing"""
        self.nmn3_model_tst = NMN3Model(
            self.image_feat_batch, self.input_seq_batch,
            self.seq_length_batch, T_decoder=T_decoder,
            num_vocab_txt=self.num_vocab_txt, embed_dim_txt=embed_dim_txt,
            num_vocab_nmn=self.num_vocab_nmn, embed_dim_nmn=embed_dim_nmn,
            lstm_dim=lstm_dim, num_layers=num_layers,
            assembler=self.assembler,
            encoder_dropout=False,
            decoder_dropout=False,
            decoder_sampling=False,
            num_choices=self.num_choices)

        self.snapshot_saver = tf.train.Saver(max_to_keep=None)
        self.snapshot_saver.restore(self.sess, self.snapshot_file)
        self.input_seq = np.zeros((T_encoder, 1), np.int32)
        self.seq_length = np.zeros(1, np.int32)
        self.image_feat = np.zeros((1, H_feat, W_feat, D_feat), np.float32)
        self.image_feat = self.pool5_val

    def run_test(self):
        print('Running test...')
        self.layout_valid = 0
        self.answer_word_list = self.answer_dict.word_list
        self.output_answers = []
        self.question_inds = [self.vocab_dict.word2idx(w) for w in self.question_tokens]
        self.seq_length = len(self.question_inds)

        self.input_seq[:self.seq_length, 0] = self.question_inds
        input_seq = self.input_seq
        seq_length = [self.seq_length]
        image_feat = self.image_feat
        self.h = self.sess.partial_run_setup(
            [self.nmn3_model_tst.predicted_tokens, self.nmn3_model_tst.scores],
            [self.input_seq_batch, self.seq_length_batch, self.image_feat_batch,
             self.nmn3_model_tst.compiler.loom_input_tensor, self.expr_validity_batch])

        """Part 0 & 1: Run Convnet and generate module layout"""
        self.tokens = self.sess.partial_run(self.h, self.nmn3_model_tst.predicted_tokens,
                                            feed_dict={self.input_seq_batch: input_seq,
                                                       self.seq_length_batch: seq_length,
                                                       self.image_feat_batch: image_feat})

        """ Assemble the layout tokens into network structure"""
        expr_list, expr_validity_array = self.assembler.assemble(self.tokens)
        self.layout_valid += np.sum(expr_validity_array)

        """Build TensorFlow Fold input for NMN"""
        expr_feed = self.nmn3_model_tst.compiler.build_feed_dict(expr_list)
        expr_feed[self.expr_validity_batch] = expr_validity_array

        """Part 2: Run NMN and learning steps"""
        self.scores_val = self.sess.partial_run(self.h, self.nmn3_model_tst.scores, feed_dict=expr_feed)

        """Get answer"""
        self.predictions = np.argmax(self.scores_val, axis=1)
        self.output_answer = [self.answer_word_list[p] for p in self.predictions]
        print("Answer:", self.output_answer[0])
        self.answer = self.output_answer[0]

        return self.answer


if __name__ == "__main__":

    while True:
        try:
            PepperDescribeScn(IP, PORT)

        except KeyboardInterrupt:
            print("Quitting...")
            PepperDescribeScn.tts.say("Thanks for testing me, I hope I will do better next time.")
            sys.exit(1)
