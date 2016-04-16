import pyaudio, wave, sys, os, time, re
import numpy as np

save_dir = sys.argv[1]

# Check directory 
if not os.path.isdir(save_dir):
    os.mkdir(save_dir)

# for files ending in '.wav', 
cur_f_names = [re.findall('(.*).wav',x)[0] for x in os.listdir(save_dir)]

# unless there aren't any
if not len(cur_f_names) == 0:
    # find all that can be int()'d
    int_f_names = []
    for xi in cur_f_names:
        try:
            int_f_names.append(xi)
        except:
            pass
    int_f_names = np.array(int_f_names)

    # and then sort, and find largest. This is starting number
    start_int = int(int_f_names[np.argsort(int_f_names)[-1]]) + 1
else:
    start_int = 0

cur_int = start_int

# Setup
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 2
RATE = 44100
RECORD_SECONDS = 5
# WAVE_OUTPUT_FILENAME = '%05i.wav' % start_int

p = pyaudio.PyAudio()

stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                frames_per_buffer=CHUNK)

while(True):
    try:
        WAVE_OUTPUT_FILENAME = '%05i.wav' % cur_int
        stream.start_stream()
        print("* recording")

        start=time.time()
        print 'Saving audio to %s in directory %s' % (WAVE_OUTPUT_FILENAME, save_dir)
        raw_input('Press enter when finished speaking..')
        # time.sleep(0.1)
        end = time.time()

        frames = stream.read(int(RATE * (end-start)))

        print("* done recording")
        print

        stream.stop_stream()
        # stream.close()
        # p.terminate()

        wf = wave.open(save_dir+WAVE_OUTPUT_FILENAME, 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(frames)
        wf.close()

        cur_int += 1
    except KeyboardInterrupt:
        sys.exit()
