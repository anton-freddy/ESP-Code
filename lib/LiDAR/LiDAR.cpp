#include <LiDAR.h>

TF_LUNA::TF_LUNA(int16_t address, uint16_t frame_rate, int LiDAR_id){
    this->address = address;
    this->LiDAR_id = LiDAR_id;
    this->frame_rate = frame_rate;
}

bool TF_LUNA::begin(){
      while (bool flag = false)
  {
    if (luna.Set_Frame_Rate(frame_rate, this->address))
    {

      flag = true;
    }
    else
    {
      send_ERROR(this->LiDAR_id, 0x00);
        return false;
      break;
    }
  }

  while (bool flag = false)
  {
    if (luna.Set_Trig_Mode(this->address))
    {

      flag = true;
    }
    else
    {
      send_ERROR(this->LiDAR_id, 0x01);
      return false;
      break;
    }
  }
  return true;
}

int16_t TF_LUNA::getDistance(){
    int16_t temp = 0;
        int i = 0;
  while (i < 10)
  {
    if (luna.Set_Trigger(this->address))
    {
      break;
    }
    i++;
  }
  if (i == 10)
  {
    send_ERROR(this->LiDAR_id, 0x02);
    return 0;
  }
  else
  {
    i = 0;
    while (i < 10)
    {
      if (luna.getData(temp, this->address))
      {
        return temp;
      }
      i++;
    }
    if (i == 10)
    {
      send_ERROR(this->LiDAR_id, 0x03);
      return 0;
    }
  }
}