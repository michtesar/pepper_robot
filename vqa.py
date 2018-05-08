from __future__ import absolute_import, division, print_function

import argparse, sys
parser = argparse.ArgumentParser()
parser.add_argument('--gpu_id', type=int, default=0)
args = parser.parse_args()
gpu_id = args.gpu_id  # set GPU id to use
import os; os.environ["CUDA_VISIBLE_DEVICES"] = str(gpu_id)
sys.path.append('./exp_clevr/data/')
import numpy as np
import text_processing
from numpy import array
import tensorflow as tf
from nmn3_assembler import Assembler
from nmn3_model import NMN3Model
sys.path.append('../')
sys.path.append('./')
from glob import glob
import skimage.io
import skimage.color
from models_clevr import vgg_net
import qi, cv2
import speech_recognition as sr
import time, Image
import paramiko, os

IP = "paprika.local"
PORT = 9559

# Extract image features
vgg_net_model = './exp_clevr/tfmodel/vgg_net/vgg_net.tfmodel'



#################### Pepper stuff - he takes picture and listens to question ######################################
session = qi.Session()
session.connect("tcp://" + IP + ":" + str(PORT))

awareness = session.service('ALBasicAwareness')
motion_service = session.service("ALMotion")
photo_service = session.service("ALPhotoCapture")
awareness.pauseAwareness()
motion_service.angleInterpolationWithSpeed(["HeadYaw", "HeadPitch"], [-0.1, 0.4], 0.2)
motion_service.setStiffnesses(["HeadYaw", "HeadPitch"], [0.0, 0.0])
#photo_service.force_connect()
photo_service.setCameraID(0)
photo_service.setPictureFormat("jpg")
photo_service.takePicture('/home/nao/photo/', 'scene')

# SSH to nao and copy image, view it 
paramiko.util.log_to_file('/tmp/paramiko.log')
paramiko.util.load_host_keys(os.path.expanduser('~/.ssh/known_hosts'))
remote_images_path = '/home/nao/photo/scene.jpg'
local_path = '/home/gabi/Desktop/scene1.jpg'
ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(
            paramiko.AutoAddPolicy())
#ssh.connect(PEPPER_IP, username="nao", password="ciirc07032018")
ssh.connect(IP, username="nao", password="nao")
sftp = ssh.open_sftp()
sftp.get(remote_images_path, local_path)
sftp.close()
ssh.close()
img = cv2.imread('/home/gabi/Desktop/scene1.jpg', 1)
resized_img = cv2.resize(img, (480,320)) 
img_f = img[20:340, 0:480 ]
cv2.imshow("Robot Eyes", img_f)
cv2.waitKey(3)

#img.save('/home/gabi/Desktop/scene.jpg')
recorder = session.service("ALAudioRecorder")
tts = session.service("ALTextToSpeech")
awareness.resumeAwareness()
tts.say("i looked at the scene. ask me")
filename = "/home/nao/test.wav"
recorder.startMicrophonesRecording(filename, "wav", 48000, (0, 0, 1, 0))

time.sleep(4)
recorder.stopMicrophonesRecording()

paramiko.util.log_to_file('/tmp/paramiko.log')
paramiko.util.load_host_keys(os.path.expanduser('~/.ssh/known_hosts'))
remote_images_path = '/home/nao/test.wav'
local_path = '/home/gabi/wav/test.wav'
ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(
            paramiko.AutoAddPolicy())
#ssh.connect(PEPPER_IP, username="nao", password="ciirc07032018")
ssh.connect(IP, username="nao", password="nao")
sftp = ssh.open_sftp()
sftp.get(remote_images_path, local_path)
sftp.close()
ssh.close()
r = sr.Recognizer()
speech = sr.AudioFile('/home/gabi/wav/test.wav')
with speech as source:
    audio = r.record(source)

    recognised = r.recognize_google(audio)
    recognised = recognised.replace("wall", "ball")
    recognised = recognised.replace("Bull", "ball")
recognised = recognised.replace("colour", "color")
print(recognised)
###################################################################################

# Image file to recognize
image_basedir = '/home/gabi/Desktop/scene.jpg'

H = 320
W = 480

image_batch = tf.placeholder(tf.float32, [1, H, W, 3])
pool5 = vgg_net.vgg_pool5(image_batch, 'vgg_net')
saver = tf.train.Saver()
sess = tf.Session(config=tf.ConfigProto(gpu_options=tf.GPUOptions(allow_growth=True)))
saver.restore(sess, vgg_net_model)
#im = skimage.io.imread(image_basedir)[..., :3]
#im = skimage.io.img_f[..., :3]
im_val = (img_f[np.newaxis, ...]-vgg_net.channel_mean)
pool5_val = pool5.eval({image_batch: im_val}, sess)
sess.close() 
tf.reset_default_graph()

# Initialize model session
sess = tf.Session(config=tf.ConfigProto(
    gpu_options=tf.GPUOptions(allow_growth=True),
    allow_soft_placement=False, log_device_placement=False))
    
# Module parameters
H_feat = 10
W_feat = 15
D_feat = 512
embed_dim_txt = 300
embed_dim_nmn = 300
lstm_dim = 512
num_layers = 2
T_encoder = 45
T_decoder = 20
prune_filter_module = True

snapshot_file = './exp_clevr/tfmodel/clevr_rl_gt_layout/00050000' 

# Data files
vocab_question_file = './exp_clevr/data/vocabulary_clevr.txt'
vocab_layout_file = './exp_clevr/data/vocabulary_layout.txt'
vocab_answer_file = './exp_clevr/data/answers_clevr.txt'



#raw_question = raw_input("Ask question: ")
raw_question = recognised
question_tokens = text_processing.tokenize(raw_question)

assembler = Assembler(vocab_layout_file)

vocab_dict = text_processing.VocabDict(vocab_question_file)

answer_dict = text_processing.VocabDict(vocab_answer_file)
	
num_vocab_txt = vocab_dict.num_vocab
num_vocab_nmn = len(assembler.module_names)
num_choices = answer_dict.num_vocab
# Network inputs
input_seq_batch = tf.placeholder(tf.int32, [None, None])
seq_length_batch = tf.placeholder(tf.int32, [None])
image_feat_batch = tf.placeholder(tf.float32, [None, H_feat, W_feat, D_feat])
expr_validity_batch = tf.placeholder(tf.bool, [None])

# The model for testing
nmn3_model_tst = NMN3Model(
    image_feat_batch, input_seq_batch,
    seq_length_batch, T_decoder=T_decoder,
    num_vocab_txt=num_vocab_txt, embed_dim_txt=embed_dim_txt,
    num_vocab_nmn=num_vocab_nmn, embed_dim_nmn=embed_dim_nmn,
    lstm_dim=lstm_dim, num_layers=num_layers,
    assembler=assembler,
    encoder_dropout=False,
    decoder_dropout=False,
    decoder_sampling=False,
    num_choices=num_choices)

snapshot_saver = tf.train.Saver(max_to_keep=None)
snapshot_saver.restore(sess, snapshot_file)
actual_batch_size = 1
input_seq = np.zeros((T_encoder, actual_batch_size), np.int32)
seq_length = np.zeros(actual_batch_size, np.int32)
image_feat = np.zeros((actual_batch_size, H_feat, W_feat, D_feat), np.float32)
image_feat = pool5_val


def run_test():
    print('Running test...')
    layout_valid = 0
    answer_word_list = answer_dict.word_list
    output_answers = []
    #for batch in dataset_tst.batches():
        # set up input and output tensors
    question_inds = [vocab_dict.word2idx(w) for w in question_tokens]

    seq_length = len(question_inds)

    input_seq[:seq_length,0] = question_inds
    seq_length = [seq_length]
    h = sess.partial_run_setup(
            [nmn3_model_tst.predicted_tokens, nmn3_model_tst.scores],
            [input_seq_batch, seq_length_batch, image_feat_batch,
             nmn3_model_tst.compiler.loom_input_tensor, expr_validity_batch])
    
        # Part 0 & 1: Run Convnet and generate module layout
    tokens = sess.partial_run(h, nmn3_model_tst.predicted_tokens,
 	feed_dict={input_seq_batch: input_seq,
                   seq_length_batch: seq_length,
                   image_feat_batch: image_feat})

        # Assemble the layout tokens into network structure
    expr_list, expr_validity_array = assembler.assemble(tokens)
    layout_valid += np.sum(expr_validity_array)
        # Build TensorFlow Fold input for NMN
    expr_feed = nmn3_model_tst.compiler.build_feed_dict(expr_list)
    expr_feed[expr_validity_batch] = expr_validity_array

        # Part 2: Run NMN and learning steps
    scores_val = sess.partial_run(h, nmn3_model_tst.scores, feed_dict=expr_feed)

        # compute accuracy
    predictions = np.argmax(scores_val, axis=1)
    output_answers += [answer_word_list[p] for p in predictions]

    print(output_answers[0])
    # pepper answers
    tts.say(output_answers[0])
run_test()
