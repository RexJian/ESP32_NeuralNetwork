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
#### [SwitchCombineModel.c](https://github.com/RexJian/ESP32_NeuralNetwork/blob/main/ESP32_CombineModel/SwitchCombineModel.c) : Recognize handwritten equation
The program operates according to the following steps  
  
1. Using SPI communication connect ADXL345 and ESP32  
  
2. When the DIP switch turn on , it would record ADXL345 3 axis data  
  
3. Data would be through a series of preprocessing acctions like adjust the data length and normalize data  
  
4. When the DIP switch turn off , data would be the input of model , then the model would recognize the data to a number  
  
5. Get the complete equation and calculate the equation answer .  



