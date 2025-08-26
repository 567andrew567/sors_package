# SROS_package

## 說明

### sros_package
* 這個package中包含金鑰交換、AES加密、pathORAM_like混淆演算法等工具

#### kyber_server
* 放至於server中，用於與user及bot進行金鑰交換，使用該金鑰用於後續aes加密通訊
* 每一個bot及user都需要運行一個kyber_server進行服務，在預設情境中server需要開4個不同的kyber_server
* 使用方式：
    * 直接運行即可
    * 需要2個參數，`{topic name}`及`{key path}`，`{topic name}`指定service的topic名稱，`{key path}`指定金鑰的存放路徑
    * ex:
    ```bash=
    ros2 run sros_package kyber_server bot1_kyber kyber_keys/bot1_server.key
    ```
* **注意事項**
    * 每次金鑰交換拿到的金鑰都不一樣，注意server發送指令時用的金鑰是否是最新的

#### kyber_client
* 放至於bot及user端，透過此功能與kyber_server進行金鑰交換，使用該金鑰用於後續aes加密通訊
* **每台機器只須進行一次**
* 使用方式：
    * import kyber_client至需要運行金鑰交換的部份
    * 直接執行kyber_client()即可
    * 需要2個參數，`{topicname}`及`{keypath}`，`{topic name}`，`{topic_name}`帶入server指定的service的topic名稱，`{key_path}`指定金鑰的存放路徑
    * ex:
    ```python=
    kyber_client('bot1','kyber_keys/bot1_client.key')
    ```

#### AES_tools
* 提供AES加密與解密工具，有cbc與gcm兩種模式，麻煩使用gcm模式，以下說明以gcm模式為主
* 初始化：
    * class需要帶入金鑰儲存位置
    * ex:
    ```python=
    from sros_package.AES_tools import AES_tools
    aes = AES_tools(key_path)
    ```
* 加密：
    * encrypt_obj_gcm(self,data)
    * `data`傳入要加密的物件，任何型態皆可，會以pickle序列化該物件後進行base64編碼再加密
    * 回傳一個`json`字串包含密文與驗證用的資訊
    * 可使用`std_msgs/String.msg`傳送加密訊息
* 解密：
    * decrypt_obj_gcm(self,input_json)
    * `input_json`傳入要解密的物件，會進行解密及驗證，再以pickle去序列化該物件，並回傳原始的物件

#### AES_topic
* 提供aes加密傳輸的範例，並在publisher_ORAM有使用到其中的AES_publisher
* AES_publisher
    * 負責發送已加密的訊息

* AES_subscriber
    * 接收並解密密文的範例，可參考此作法實做接收端的程式

#### publisher_ORAM
* 運行pathORAM_like混淆演算法並傳輸加密訊息給所有bot
* **這項功能只須實做於server上**
* **控制3台bot的部份皆需實做一次**
* 初始化：
    * class需要帶入`{topic_list}`、`{key_path}`、`{target}`，`{topic_list}`須帶入一個list，包含3台bot接收控制指令的topic名稱，`{key_path}`目標bot所使用的金鑰位置，`{target}`目標bot所使用的接收控制指令的topic名稱
    * ex:
    ```python=
    from sros_package.publisher_ORAM import ORAM_Node
    topic_list = ["bot1_topic","bot2_topic","bot3_topic"]
    key_path_dict = "kyber_keys/bot1_client.key"
    oram_node = ORAM_Node(topic_list, key_path_dict, "bot1_topic")
    ```
* 使用方式：
    * 使用`send_data(data)`發送訊息，`data`傳入需要傳送給目標的指令資料，會執行aes加密後傳送給目標topic及以pathORAM_like混淆演算法選擇多個其他topic
    * ex:
    ```python=
    from sros_package.publisher_ORAM import ORAM_Node
    topic_list = ["bot1_topic","bot2_topic","bot3_topic"]
    key_path_dict = "kyber_keys/bot1_client.key"
    oram_node = ORAM_Node(topic_list, key_path_dict, "bot1_topic")
    data = 'test'
    oram_node.send_data(data)
    ```
* 初始化前，**請勿在同個程式中使用`rclpy.init(args=None)`**，避免ORAM_Node內部的multiprocessing會有問題

#### 注意事項
* 建議把sros_package放入workpsace/scr中，在其他package中使用以下範例import要用的功能
    ```python=
    from sros_package.AES_tools import AES_tools
    from sros_package.publisher_ORAM import ORAM_Node
    from sros_package.kyber_client import kyber_client
    ```

### interfaces

* 用於kyber server與kyber client中自訂service傳送的訊息類型

#### 注意事項
* 必須要將interfaces資料夾放入要執行的ros2的workpsace/scr中一起編譯

## 範例

### sros_kyber_aes_oram_client_test.py
* 與kyber_server金鑰交換後將訊息透過publisher_ORAM發送
* 使用kyber_client()時是使用multiprocessing在另一個至行序中執行，避免在service啟動前的rclpy.init(args=None)影響publisher_ORAM的運行

### subscriber_aes_test.py

* 使用AES_subscriber接收並解密密文的範例，可參考此作法實做接收端的程式