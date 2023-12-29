# **Using ESP32 and ADXL345,combine a neural network to create a simple calculator**
In the project, we aim to use the ADXL345 to collect data. Subsequently, we will utilize this data to train a simple neural network. Upon completing the model,  use it for recognizing handwritten equations. Finally, the complete equation and its corresponding answer will be displayed on the terminal
## Content
* Develope Framework and IDE
* File Description
* Demo Video Link
* Author
### Develope Framework and IDE
[VS Code](https://code.visualstudio.com/) : Connect Esp32 and ADXL345  
  
[Visual Studio](https://visualstudio.microsoft.com/zh-hant/) : Using collect data train model  
  
[ESP-IDF Branch 5.0](https://github.com/espressif/esp-idf) :  Provide a comprehensive environment with rich features, RTOS support, and continuous updates.

### File Description
#### [ADXL345_CollectData.c](https://github.com/RexJian/ESP32_NeuralNetwork/blob/main/collect_data/ADXL345_CollectData.c) : Collect data from the ADXL345 and export the data as a log file  
The program operates according to the following steps  
  
1. Connect the ADXL345 to the ESP32 using SPI communication  
  
2. When the DIP switch is turned on, it will record ADXL345 3-axis data.  
  
3. The data will undergo a series of preprocessing actions, such as adjusting the data length and normalizing the data.  
  
4. When the DIP switch is turned off, display all data on Putty.

5. Export the Putty log file to the specified data path.   


#### [Train Folder](https://github.com/RexJian/ESP32_NeuralNetwork/tree/main/Train) : Train Model  
The program operates according to the following steps

1. Set hyperparameter, such as hidden layer neurons number and learning rate.   

2. Initial model weights.  

3. Load 3-axis train data log file.

4. Start train.  
  
5. Load 3-axis test data log file.
  
6. Start test.   

7. Show the weights and test result.
   
   
#### [SwitchCombineModel.c](https://github.com/RexJian/ESP32_NeuralNetwork/blob/main/ESP32_CombineModel/SwitchCombineModel.c) : Recognize handwritten equation
The program operates according to the following steps  
  
1. Connect the ADXL345 to the ESP32 using SPI communication  
  
2. When the DIP switch is turned on, it will record ADXL345 3-axis data.  
  
3. The data will undergo a series of preprocessing actions, such as adjusting the data length and normalizing the data.  
  
4. When the DIP switch is turned off, the data serves as the input for the model, which then recognizes the data and outputs a corresponding number.  
  
5. Retrieve the entire equation and compute the corresponding answer.  


