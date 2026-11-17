# 所有播放類 action 共用的 feedback

# STAGE_LOADING：正在載入內容（讀檔、建立 ONNX session 等），還沒有切換，機器人繼續原本的動作
# STAGE_PLAYING：已切換，只在 wait_until_finished 時持續發布
uint8 STAGE_LOADING=0
uint8 STAGE_PLAYING=1
uint8 stage

# 開始播放之後經過的時間（秒）；載入中為 0
float64 elapsed
