#include "esp_timer.h"
#include "esp_camera.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "fd_forward.h"
#include "fr_forward.h"
#include "fr_flash.h"
#include "FirebaseESP32.h"
FirebaseData firebaseData;
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "Base64.h"

const char* ssid = "Giang Tuan";
const char* password = "73759899";
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
String FIREBASE_HOST = "esp32-doorbell-hutech.firebaseio.com";
String FIREBASE_AUTH = "SB6fUE9w9HW3h2ZJtoEfxJpnnrbY7O9ExMXniw9c";
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#define ENROLL_CONFIRM_TIMES 5
#define FACE_ID_SAVE_NUMBER 7
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
camera_fb_t * fb = NULL;
//long current_millis;
long last_detected_millis = 0;
void app_facenet_main();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  uint8_t *image;
  box_array_t *net_boxes;
  dl_matrix3d_t *face_id;
} img_process_result;
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
static inline mtmn_config_t app_mtmn_config()
{
  mtmn_config_t mtmn_config = {0};
  mtmn_config.type = FAST;
  mtmn_config.min_face = 80;
  mtmn_config.pyramid = 0.707;
  mtmn_config.pyramid_times = 4;
  mtmn_config.p_threshold.score = 0.6;
  mtmn_config.p_threshold.nms = 0.7;
  mtmn_config.p_threshold.candidate_number = 20;
  mtmn_config.r_threshold.score = 0.7;
  mtmn_config.r_threshold.nms = 0.7;
  mtmn_config.r_threshold.candidate_number = 10;
  mtmn_config.o_threshold.score = 0.7;
  mtmn_config.o_threshold.nms = 0.7;
  mtmn_config.o_threshold.candidate_number = 1;
  return mtmn_config;
}
mtmn_config_t mtmn_config = app_mtmn_config();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
face_id_name_list st_face_list;
static dl_matrix3du_t *aligned_face = NULL;
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  char enroll_name[ENROLL_NAME_LEN];
}resp_value;

resp_value st_name;
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif;lp/
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  if (WiFi.status() == WL_CONNECTED) {
    char* apssid = "ESP32-CAM";
    char* appassword = "12345678";         //AP password require at least 8 characters.
    Serial.println(""); 
    Serial.print("Camera Ready! Use 'http://");
    Serial.print(WiFi.localIP());
    Serial.println("' to connect");
    WiFi.softAP((WiFi.localIP().toString()+"_"+(String)apssid).c_str(), appassword);            
  }
  else {
    Serial.println("Connection failed");
    return;
  }
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- 
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
  Firebase.setMaxRetry(firebaseData, 3);
  Firebase.setMaxErrorQueue(firebaseData, 30); 
  Firebase.enableClassicRequest(firebaseData, true);
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------  
  app_facenet_main();
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void app_facenet_main()
{
  face_id_name_init(&st_face_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);
  aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
  read_face_id_from_flash_with_name(&st_face_list);
 }
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
static inline int do_enrollment(face_id_name_list *face_list, dl_matrix3d_t *new_id)
{
  Serial.println("START ENROLLING");
  int left_sample_face = enroll_face_id_to_flash_with_name(face_list, new_id, st_name.enroll_name);
  
  /*ESP_LOGD(TAG, "Face ID %s Enrollment: Sample %d",
           st_name.enroll_name,
           ENROLL_CONFIRM_TIMES - left_sample_face);*/
  
  if((ENROLL_CONFIRM_TIMES - left_sample_face) != 0)
    Firebase.setString(firebaseData, "/enroll/status", "running");
  else if((ENROLL_CONFIRM_TIMES - left_sample_face) == 0)
    Firebase.setString(firebaseData, "/enroll/status", "done");
  return left_sample_face;
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void takePhoto(camera_fb_t * fb, String face_name)
{
  String jsonData = "{\"name\":\"" + face_name + "\" ,\"photo\":\"" + Photo2Base64(fb) +"\"}";
  String photoPath = "/esp32-cam"; 
  String controlPath = "/controlStatus/takePhotoBtn";
  if(Firebase.setString(firebaseData, controlPath, "off"))
    Serial.println("Set button off");
  if (Firebase.setJSON(firebaseData, photoPath, jsonData)) {
    Serial.println("Push Json done");
  } else {
    Serial.println(firebaseData.errorReason());
  }
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
String checkBtnStatus(String path)
{
  if (Firebase.getString(firebaseData, path ))
    {
      if (firebaseData.dataType() == "string")
        return firebaseData.stringData();    
  }
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void loop() 
{
    String do_enroll = checkBtnStatus("/controlStatus/doEnrollment");
    String physicBtn = digitalRead(13) == LOW ? "on" : "of";
    if (physicBtn == "on" || checkBtnStatus("/controlStatus/takePhotoBtn") == "on" || do_enroll == "on" )//Start
    {
      Serial.println("1");
      dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, 320, 240, 3);      
      img_process_result out_res = {0};
      out_res.image = image_matrix->item;
      fb = esp_camera_fb_get();
      out_res.net_boxes = NULL;
      out_res.face_id = NULL;

      fmt2rgb888(fb->buf, fb->len, fb->format, out_res.image);
      Serial.println("2");
      out_res.net_boxes = face_detect(image_matrix, &mtmn_config);
      Serial.println("3");
      if (out_res.net_boxes) //detect
      {
        Serial.println("4");
        if (align_face(out_res.net_boxes, image_matrix, aligned_face) == ESP_OK)
        {
          Serial.println("5");
          out_res.face_id = get_face_id(aligned_face);
          Serial.println("6");
          last_detected_millis = millis();
          if (st_face_list.count > 0 && (checkBtnStatus("/controlStatus/takePhotoBtn") == "on" || physicBtn == "on "))//recog
          {
            face_id_node *f = recognize_face_with_name(&st_face_list, out_res.face_id);
            if (f)
            {
              Serial.println("RECOGNISED ");
              takePhoto(fb, f->id_name);   
            }
            else
            {
              Serial.println("FACE NOT RECOGNISED");
              takePhoto(fb, "FACE NOT RECOGNISED");
            }
          }
          if (do_enroll == "on")//enroll
          {
            String f_name = checkBtnStatus("/enroll/name");
            char person[FACE_ID_SAVE_NUMBER * ENROLL_NAME_LEN] = {0,};
            f_name.substring(8).toCharArray(person, sizeof(person));
            memcpy(st_name.enroll_name, person, strlen(person) + 1);
            int left_sample_face = do_enrollment(&st_face_list, out_res.face_id);
            //char enrolling_message[64];
            //sprintf(enrolling_message, "SAMPLE NUMBER %d FOR %s", ENROLL_CONFIRM_TIMES - left_sample_face, st_name.enroll_name);
            if (left_sample_face == 0)
            {
              //char captured_message[64];
              //Serial.printf(captured_message, "FACE CAPTURED FOR %s", st_face_list.tail->id_name);
              if(Firebase.setString(firebaseData, "/controlStatus/do_Enrollment", "off"))
                Serial.println("Set do_enrollment off");
            }
          }
          dl_matrix3d_free(out_res.face_id);
        }
      }
      else 
      {
        if( do_enroll != "on")
        {
        Serial.println("NO FACE DETECTED");
        takePhoto(fb, "NO FACE DETECTED");
        }
      }
    }
    esp_camera_fb_return(fb);
    fb = NULL;
    physicBtn = "off";
}
String Photo2Base64(camera_fb_t * fb) {
    String imageFile = "data:image/jpeg;base64,";
    char *input = (char *)fb->buf;
    char output[base64_enc_len(3)];
    for (int i=0;i<fb->len;i++) {
      base64_encode(output, (input++), 3);
      if (i%3==0) imageFile += urlencode(String(output));
    }

    esp_camera_fb_return(fb);
    
    return imageFile;
}

//https://github.com/zenmanenergy/ESP8266-Arduino-Examples/
String urlencode(String str)
{
    String encodedString="";
    char c;
    char code0;
    char code1;
    char code2;
    for (int i =0; i < str.length(); i++){
      c=str.charAt(i);
      if (c == ' '){
        encodedString+= '+';
      } else if (isalnum(c)){
        encodedString+=c;
      } else{
        code1=(c & 0xf)+'0';
        if ((c & 0xf) >9){
            code1=(c & 0xf) - 10 + 'A';
        }
        c=(c>>4)&0xf;
        code0=c+'0';
        if (c > 9){
            code0=c - 10 + 'A';
        }
        code2='\0';
        encodedString+='%';
        encodedString+=code0;
        encodedString+=code1;
        //encodedString+=code2;
      }
      yield();
    }
    return encodedString;
}
