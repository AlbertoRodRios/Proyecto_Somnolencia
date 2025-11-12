import tensorflow as tf
from tensorflow import keras
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.utils import load_img, img_to_array
import numpy as np

# 1. Load Data with Augmentation
# (Rescale, shear, zoom, flip for training)
train_datagen = ImageDataGenerator(rescale=1./255, shear_range=0.2, ...)
training_set = train_datagen.flow_from_directory(
    'Fotos_Training',
    target_size=(240, 100),
    color_mode='grayscale',
    class_mode='binary')

# (Rescale only for testing)
test_datagen = ImageDataGenerator(rescale=1./255)
test_set = test_datagen.flow_from_directory('Fotos_Test', ...)

# 2. Build the CNN Model
cnn = tf.keras.models.Sequential([
    # Input: [240, 100, 1] (Grayscale)
    keras.layers.Conv2D(filters=16, kernel_size=3, activation='relu', 
                         input_shape=[240, 100, 1]),
    keras.layers.MaxPool2D(pool_size=2),
    
    keras.layers.Conv2D(filters=16, kernel_size=3, activation='relu'),
    keras.layers.MaxPool2D(pool_size=2),
    
    keras.layers.Flatten(),
    keras.layers.Dense(units=32, activation='relu'), # Hidden layer
    keras.layers.Dense(units=1, activation='sigmoid') # Output layer
])

# 3. Compile and Train the Model
cnn.compile(optimizer='adam', loss='binary_crossentropy', metrics=['accuracy'])
cnn.fit(training_set, validation_data=test_set, epochs=25)

# 4. Make a Single Prediction
image = load_img('single_image.pgm', target_size=(240, 100))
image_arr = img_to_array(image)
image_batch = np.expand_dims(image_arr, axis=0) # Create a batch of 1

result = cnn.predict(image_batch)
prediction = 'despierto' if result[0][0] == 1 else 'dormido'
print(prediction)