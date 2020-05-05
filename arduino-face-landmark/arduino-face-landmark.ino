#include <Sipeed_OV2640.h>
#include <Sipeed_ST7789.h>
#include <SD.h>
#include <Maix_KPU.h>
extern "C" {
  #include "region_layer.h"
  #include "prior.h"
}
#define KMODEL_SIZE (4220 * 1024)

SPIClass        spi_(SPI0);
Sipeed_ST7789   lcd(320, 240, spi_);
Sipeed_OV2640   camera(320, 240, PIXFORMAT_RGB565);
KPUClass        _kpu;
uint8_t*        model;

static region_layer_t rl;
size_t pred_box_size, pred_landm_size, pred_clses_size;
static box_info_t boxes;
static float variances[2]= {0.1, 0.2};
float *pred_box;
float *pred_landmark;
float *pred_class;

const char* mask_model_name = "mark.net";

void screen_log(const char* text){
  lcd.fillScreen(COLOR_RED);
  lcd.setCursor(100,100);
  lcd.print(text);
}

void drawboxes(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint32_t nclass, float prob, uint32_t *landmark, uint32_t landm_num) {
    if (x1 >= 320) x1= 319;
    if (x2 >= 320) x2= 319;
    if (y1 >= 224) y1= 223;
    if (y2 >= 224) y2= 223;

    lcd.drawRect(x1, y1, x2 - x1 , y2 - y1, COLOR_RED);
    for (uint32_t i= 0; i < landm_num; i++) {
      int lx = landmark[2 * i];
      int ly = landmark[1 + 2 * i];
      if (lx >= 320) lx= 319;
      if (ly >= 224) ly= 223;
      lcd.fillCircle(lx,ly , 4, COLOR_GREEN);
    }
}
void draw_boxes(box_info_t *bx){
  uint32_t landmark[bx->landm_num * 2];
  float prob;
  for (uint32_t i= 0; i < bx->row_idx; i++) {
      prob= bx->box[bx->box_len * i + bx->crood_num];
      if (prob > bx->obj_thresh) {
          float *b= &bx->box[bx->box_len * i];
          uint32_t x1= (b[0] - (b[2] / 2)) * bx->in_w;
          uint32_t y1= (b[1] - (b[3] / 2)) * bx->in_h;
          uint32_t x2= (b[0] + (b[2] / 2)) * bx->in_w;
          uint32_t y2= (b[1] + (b[3] / 2)) * bx->in_h;
          Serial.printf("%d\t%d\t%d\t%d\t%f\t\r\n", x1, y1, x2, y2, b[4]);
          for (uint32_t j= 0; j < bx->landm_num; j++) {
              landmark[0 + j * 2]= (uint32_t)(b[5 + j * 2] * bx->in_w);
              landmark[1 + j * 2]= (uint32_t)(b[6 + j * 2] * bx->in_h);
          }
          drawboxes(x1, y1, x2, y2, i, prob, landmark, bx->landm_num);
      }
  }
}
void setup() {
  File model_file;
  Serial.begin(115200);
  if(!camera.begin()){
    Serial.println("Cannot init camera");
    return;
  }
  //if(!lcd.begin(20000000, COLOR_RED, 2)){//uncomment this, if use ST7789 240x240
  if(!lcd.begin(15000000, COLOR_RED)){ //uncomment this, if use ST7789 320x240 
    Serial.println("Cannot init display");
    return;
  }
  camera.run(true);
  lcd.setTextSize(2);
  lcd.setTextColor(COLOR_WHITE);
  if (!SD.begin()) 
  {
    screen_log("No SD Card");
    return;
  }
  model_file = SD.open(mask_model_name);
  if (!model_file){
    screen_log("Cannot open file");
    return;
  }
  uint32_t fSize = model_file.size();
  screen_log("Loading ...");
  model = (uint8_t*)malloc(fSize);
  if(!model)
  {
      screen_log("Memmory not enough ... ");
      return;
  }
  long ret = model_file.read(model, fSize);
  model_file.close();
  if(ret != fSize)
  {
    screen_log("Read model file error");
    free(model);
    model = nullptr;
    return;
  }

  if(_kpu.begin(model) != 0)
  {
    screen_log("Cannot load model");
    free(model);
    model = nullptr;
    return;
  }
  screen_log("Load model success");
  delay(1000);
  region_layer_init(&rl, anchor, 3160, 4, 5, 1, 320, 240, 0.7, 0.4, variances);
  boxes_info_init(&rl, &boxes, 200);
}
void loop() {
  uint8_t* img;
  uint8_t* img888;
  
  img = camera.snapshot();
  if(img == nullptr || img==0){
      return;
  }
  img888 = camera.getRGB888();
  int fw = _kpu.forward(img888,DMAC_CHANNEL5);
  
  while( !_kpu.isForwardOk() );
  _kpu.getResult((uint8_t **)&pred_box, &pred_box_size,0);
  _kpu.getResult((uint8_t **)&pred_landmark,&pred_landm_size,1);
  _kpu.getResult((uint8_t **)&pred_class,&pred_clses_size,2);
  
  rl.bbox_input= pred_box;
  rl.landm_input= pred_landmark;
  rl.clses_input= pred_class;
  region_layer_run(&rl, &boxes);

  lcd.drawImage(0, 0, camera.width(), camera.height(), (uint16_t*)img);
  draw_boxes(&boxes);
  boxes_info_reset(&boxes);
}
