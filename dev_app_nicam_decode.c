#include "dev_app_nicam_decode.h"

INT32 nicam_faw_error;
INT32 nicam_need_shift;
INT32 first_flag;
INT32 nicam_pos;
INT32 nicam_remain_num;
INT32 mon_dual_stereo;
INT32 flag_l_r;
INT32 flag_lr=0;
INT32 err_nicam_data_num=0;
INT32 nicam_faw_error_count;
static UINT16 nicam_prsg_val = 0xffff;

UINT8 data_frame[BUF_SIZE_NICAM_FRAME]= {0};
UINT8  nicam_data[NICAM_LARGE_BUF_SIZE]; //0x1000 bytes
UINT8   nicam_data_fix[(NICAM_LARGE_BUF_SIZE/91+1)*91];
UINT16 nicam_decoded_data[64];
INT16 app_in_buff_left[MAX_SAMPLES_NUM];
INT16 app_in_buff_right[MAX_SAMPLES_NUM];
UINT16 __attribute__((aligned(8))) av_ifc_buff_in[APP_INPUT_BUFF_SIZE];//0x4000 0x2000
VOID  dev_app_init_nicam_decode(VOID)
{
    nicam_faw_error=0;
    nicam_need_shift=0;
    first_flag=0;
    nicam_pos=0;
    nicam_remain_num=0;
    mon_dual_stereo=-1;
    flag_l_r=-1;
    nicam_prsg_val = 0xffff;
    err_nicam_data_num=0;
    flag_lr=0;
    nicam_faw_error_count=0;
}

UINT8 big2le_bit8(UINT8 data)
{
    INT32 i=0;
    UINT8 tmp=0;
    for(; i<8; i++) {
        if((data>>(7-i))&0x1)
            tmp|=(1<<i);
    }

    return tmp;
}

VOID swap_bytes(UINT8 *p,INT32 size)
{
    INT32 i;
    for(i=0; i<size; i++) { //why not i<=size
        p[i]=big2le_bit8(p[i]);
    }
}

INT32 check_faw_and_ctrl_bit(UINT32 data)
{
    UINT8 *p;
    INT32 ret=-1;
    p=(UINT8 *)(&data);
    if((p[0]==0x4e)&&((p[1]&0x07)==0x7)) {
        switch((*(p+1)>>4)&(~(1<<3))) {
        case NICAM_STEREO:
        case NICAM_DUAL:
        case NICAM_MONO: {
            ret=0;
            break;
        }
        default: {
            break;
        }
        }
    }
    return ret;
}
INT32 check_faw(UINT32 data)
{
    UINT8 *p;
    INT32 ret=-1;
    p=(UINT8 *)(&data);
    if((p[0]==0x4e)) {
        ret=0;
    }
    return ret;
}
INT32 shift_to_l(UINT8* p_data,INT32 num)
{
    UINT8 *p,*p1;
    UINT8 tmp_data[4];
    INT32 ret=-1,i;
    UINT32 *data=(UINT32 *)(&tmp_data[0]);
    for(i=0; i<4; i++) {
        tmp_data[i]=p_data[i];
    }
    (*data) >>=(2*num);
    swap_bytes(tmp_data,4);
    ret =check_faw((*data));
    if(ret==0) {
        ret=(tmp_data[1]>>7);
    }

    return ret;
}

UINT8 reparse_data_byte(UINT8 *pdata,INT32 shift)
{
    UINT8 tmp_data[2];
    UINT16 *data=(UINT16 *)(&tmp_data[0]);
    tmp_data[0]=pdata[0];
    tmp_data[1]=pdata[1];

    return ((*data)>>(shift*2))&0x00ff;
}

INT32 redecoding_nicam_data(UINT8 *data,INT32 size,INT32 shift)
{
    INT32 i=0;
#if 1
    if((size % 91)== 0) {
        for(i=0; i<size-1; i++) {
            data[i]=reparse_data_byte((data+i),shift);
        }
        //error,the last sample is discarded
        data[i]=0;
    } else {
        size=(size/91)*91;
        for(i=0; i<size; i++) {
            data[i]=reparse_data_byte((data+i),shift);
        }
    }
#endif
    swap_bytes(data,size);
    return size;
}

INT32 confirm_faw_ok(UINT32 data)
{
    switch(data) {
    case 0x00ff:
    case 0x01fe:
    case 0x03fc:
    case 0x07f8:

    case 0x0ff0:
    case 0x1fe0:
    case 0x3fc0:
    case 0x7f80:

    case 0xff00:
    case 0xfe01:
    case 0xfc03:
    case 0xf807:

    case 0xf00f:
    case 0xe01f:
    case 0xc03f:
    case 0x807f:
        return 1;
    }
    return 0;
}

INT32 encorrect_4e_and_ctrl_bits(UINT8 *data,INT32 size,INT32 * pos,INT32* shift)
{
    INT32 j,k,i=0,ret=-1;
    UINT32 faw_flag_count=0;
    for(i=0; i<size-91*16-4; i++) {
        for(j=1; j<=4; j++) {
            ret=shift_to_l((data+i),j);

            if(ret>=0) {
                faw_flag_count=0;
                break;
            }
        }
        if(ret>=0) {
            faw_flag_count=ret;
            for(k=1; k<16; k++) {
                ret=shift_to_l((data+i+91*k),j);
                if(ret>=0) {
                    faw_flag_count|=(ret<<k);
                } else {
                    break;
                }
            }
            if((k==16)&&confirm_faw_ok(faw_flag_count)) {
                goto out;
            }

        }
    }
    return -1;
out:
    *pos=i;
    *shift=j;
    //to notice this function
    ret=redecoding_nicam_data(data+i,size-i,j);
    *shift=j;
    return ret;
}

INT32 dev_app_read_frames(UINT8* nicam_data,INT32 data_size,UINT8* nicam_data_fix)
{
    INT32 pos,ret=-1,shift,size,remain;
    INT32 j,k,i=0;
    INT32 result=data_size;
    UINT32 faw_flag_count=0;

AGIAN:
    //find 0x4e
    if(first_flag==0) {
        for(i=0; i<data_size-16*91; i++) {
#if 0
            if((nicam_data[i]==0x4e)&&(((nicam_data[i+1]&0x70)>>4)==0))//stereo
                break;
            else if((nicam_data[i]==0x4e)&&(((nicam_data[i+1]&0x70)>>4)==0x2))//dual
                break;
            else if((nicam_data[i]==0x4e)&&(((nicam_data[i+1]&0x70)>>4)==0x4))//mono
                break;
#else
            if(nicam_data[i]==0x4e) {
                faw_flag_count=(nicam_data[i+1]>>7);
                for(k=1; k<16; k++) {
                    if(nicam_data[i+k*91]==0x4e) {
                        faw_flag_count=(nicam_data[i+1]>>7)<<k;
                    } else {
                        faw_flag_count=0;
                        break;
                    }
                }
                if((k==16)&&confirm_faw_ok(faw_flag_count)) {
                    break;
                }

            }

#endif
        }
        first_flag=1;

        if(!confirm_faw_ok(faw_flag_count)) {
            mon_Event(0xb00300b0);
            nicam_faw_error=1;
            swap_bytes(nicam_data,result);

            ret=encorrect_4e_and_ctrl_bits(nicam_data,result,&pos,&shift);
            if(ret<0) {
                first_flag=0;
                nicam_faw_error=0;
                mon_Event(0xb00300bf);
                return ret;
            }
            nicam_need_shift=shift;
            for(i=0; i<ret; i++) {
                nicam_data_fix[i]=nicam_data[pos+i];
            }
            remain=(data_size-pos-ret);
            for(i=0; i<remain; i++) {
                data_frame[i]=nicam_data[pos+ret+i];
            }
            nicam_pos=i;
            return ret;
        }
        pos=i;
        mon_Event(0xb00300a0);
        //if(i!=0)
        {
            size = ((data_size-i)/91)*91;;
            for(i=0; i<size; i++) {
                nicam_data_fix[i]=nicam_data[pos+i];
            }
            ret=size;
            remain=(data_size-pos-size);
            for(i=0; i<remain; i++) {
                data_frame[i]=nicam_data[pos+size+i];
            }
            nicam_pos=i;
        }

    } else {
        if(err_nicam_data_num>0) {
            //err_nicam_data_num--;
        }
        if(nicam_faw_error==0) {
            for(i=0; i<91-nicam_pos; i++) {
                data_frame[nicam_pos+i]=nicam_data[i];
            }
            for(i=0; i<91; i++) {
                nicam_data_fix[i]=data_frame[i];
            }
#if 0
            if(nicam_data_fix[0]!=0x4e) {
                //first_flag=0;
                //nicam_faw_error=0;
                //dev_app_init_nicam_decode();
                //dev_app_reset_nicam_decode();
                if(++nicam_faw_error_count==4) {
                    err_nicam_data_num=4;
                    mon_Event(0xb00300aa);
                }
                mon_Event(0xb00300a1);
                //goto AGIAN;
            }
            if(( nicam_faw_error_count < 4 )&&( nicam_faw_error_count > 0)) {
                err_nicam_data_num--;
            }
#endif
#if 1
            if((nicam_data_fix[0]!=0x4e)&&(nicam_data_fix[91]!=0x4e)&&(nicam_data_fix[182]!=0x4e)) {
                dev_app_init_nicam_decode();
                mon_Event(0xb00300a1);
                goto AGIAN;
            }

#endif
            size = ((data_size-(91-nicam_pos))/91)*91;;
            for(i=0; i<size; i++) {
                nicam_data_fix[91+i]=nicam_data[(91-nicam_pos)+i];
            }
            ret=91+size;
            remain=(data_size-(91-nicam_pos)-size);
            for(i=0; i<remain; i++) {
                data_frame[i]=nicam_data[(91-nicam_pos)+size+i];
            }
            nicam_pos=i;
        } else {
            swap_bytes(nicam_data,result);
            remain=sizeof(data_frame)-nicam_pos;
            for(i=0; i<remain; i++) {
                data_frame[nicam_pos+i]=nicam_data[i];
            }

            redecoding_nicam_data(data_frame,sizeof(data_frame),nicam_need_shift);
            for(i=0; i<91; i++) {
                nicam_data_fix[i]=data_frame[i];
            }
#if 0
            if(nicam_data_fix[0]!=0x4e) {
                //first_flag=0;
                //nicam_faw_error=0;
                //dev_app_init_nicam_decode();
                //dev_app_reset_nicam_decode();
                if(++nicam_faw_error_count==4) {
                    err_nicam_data_num=4;
                    mon_Event(0xb00300bb);
                }
                mon_Event(0xb00300b1);
                //goto AGIAN;
            }

            if(( nicam_faw_error_count < 4 )&&( nicam_faw_error_count > 0)) {
                err_nicam_data_num--;
            }
#endif
#if 1
            if((nicam_data_fix[0]!=0x4e)&&(nicam_data_fix[91]!=0x4e)&&(nicam_data_fix[182]!=0x4e)) {
                dev_app_init_nicam_decode();
                mon_Event(0xb00300b1);
                goto AGIAN;
            }

#endif
            size=(result-remain+1);
            ret=redecoding_nicam_data((nicam_data+remain-1),size,nicam_need_shift);
            for(i=0; i<ret; i++) {
                nicam_data_fix[91+i]=nicam_data[remain-1+i];
            }
            for(i=0; i<(size-ret); i++) {
                data_frame[i]=nicam_data[remain-1+ret+i];
            }
            nicam_pos=i;
            ret=ret+91;
        }
    }

    return ret;
}

INT32 copy_data_stereo_left_right(UINT16 *in_data,INT16 *left_data,INT16 *right_data,INT32 left_count,INT32 right_count,INT32 len)
{
    int i = 0;
    for(i = 0; i < len; i+=2) {
        left_data[left_count++]= in_data[i];
        right_data[right_count++] = in_data[i+1];
    }
    return left_count;
}

INT32 copy_data_mono(UINT16 *data_in,INT16 *data_out,INT32 len,INT32 count)
{
    INT32 i=0;
    for(i=0; i<len; i++) {
        data_out[count++] =  data_in[i];
    }
    return count;
}

#if 0
INT32 confirm_left_right(UINT8* input)

{
    INT32 i,count=0;
    INT32 first=(input[1]&(1<<7));
    count++;
    for(i=1; i<8; i++) {
        if((input[i*91+1]&(1<<7))==first) {
            count++;
        } else {
            break;
        }
    }
    return (count & 0x1);
}
#else
INT32 confirm_left_or_right(UINT8  input)
{
    switch(input) {
    case 0x00:
    case 0x03:
    case 0x0f:
    case 0x3f:
    case 0xff:
    case 0xfc:
    case 0xf0:
    case 0xc0: {
        return 0;//left
    }
    case 0x01:
    case 0x07:
    case 0x1f:
    case 0x7f:
    case 0xfe:
    case 0xf8:
    case 0xe0:
    case 0x80:
        break;
    }
    return 1;//right
}
INT32 confirm_left_right(UINT8* input)
{
    UINT32 i;
    UINT8 l_r_flag_count=0;
    for(i=0; i<8; i++) {
        l_r_flag_count |=((input[i*91+1]>>7)<<(7-i));
    }
    return confirm_left_or_right(l_r_flag_count);
}

#endif

UINT8 nicam_prsg_init()
{
    nicam_prsg_val = 0xffff;
    return 0;
}

UINT8 nicam_prsg_get(VOID)
{
    UINT16 temp;
    temp = nicam_prsg_val >> 4;
    temp += nicam_prsg_val;
    temp &= 1;
    nicam_prsg_val >>= 1;
    nicam_prsg_val &= 0xff;
    nicam_prsg_val |= (temp << 8);
    return (UINT8)temp;
}

VOID nicam728_descramble(UINT8* input)
{
    UINT16 i,j,prgs;
    UINT8 temp_u8,temp_bit;
    nicam_prsg_init();
    for(i = 0; i<(NICAM_PK_LEN_U8 - NICAM_FAW_U8); i++) {

        //temp_u8 = input[i];
        //temp_u8=big2le_bit8(temp_u8);///
        //temp_bit = temp_u8;
        temp_bit = input[i];
        temp_u8 = 0;
        for(j = 0; j<8; j++) {
#if 0
            prgs = nicam_prsg_get();
            prgs += temp_bit;
            prgs &= 0x1;
            temp_u8 &= ~(1<<j);
            temp_u8 |= (prgs << j);
            temp_bit >>= 1;
#else
            prgs = nicam_prsg_get();
            prgs += (temp_bit>>(7-j));
            prgs &= 0x1;
            temp_u8 |= (prgs << (7-j));
#endif
        }
        //temp_u8=big2le_bit8(temp_u8);///
        input[i] = temp_u8;
    }

}

INT32 swap_data16(UINT8 *data,INT32 len)
{
    int i=0;
    UINT8 tmp;
    for(; i<len; i+=2) {
        tmp=data[i];
        data[i]=data[i+1];
        data[i+1]=tmp;
    }
    return  0;
}

VOID nicam728_deinterleave(UINT8* input_data,UINT16* output)
{
    UINT8 i,j,k,n;
    UINT16 temp_u16,temp_bit;
    UINT8 data[88];
    UINT16* input=(UINT16 *)(&data[0]);
    for(i=0; i<88; i++) {
        data[i]=input_data[i];
    }
#if 0
    for(i = 0 ; i < 16; i++) {
        n = 0;
        for(j = 0; j<4 ; j++) {
            temp_u16 = 0;
            temp_bit = 0;
            for(k = 0; k < 11 ; k++) {
                INT32 shift;
#if 0
                shift = 15-k-i;
#else
                shift = 4+k-i;//
#endif
                if(shift >0) {
                    temp_bit = input[(n)+k]>>(shift) & (1<<k);
                } else {
                    shift = -shift;
                    temp_bit = input[(n)+k]<<(shift) & (1<<k);
                }
                temp_u16 |= temp_bit;
            }
            n += 11;
            output[(i<<2)+j] = temp_u16;
        }

    }

#else
    for(i = 0 ; i < 16; i++) {
        for(j = 0; j<4 ; j++) {
            temp_u16 = 0;
            for(k = 0; k < 11 ; k++) {
                temp_bit=(input[j*11+k]>>(15-i))&0x1;
                temp_u16 |= (temp_bit<<k);
                //temp_u16 |= (temp_bit<<(10-k));
            }
            output[(i<<2)+j] = temp_u16;
        }

    }
#endif
}

/* check the six most significant bits to comfirm 1 number
    the function'return value is Pi
*/
UINT8 parity_check(UINT8 x)
{
    UINT8 val=0;
    x&=0x3f;
    while(x) {
        val^=x;
        x>>=1;
    }
    return val&0x1;
}

UINT8 get_Ri( UINT16 data)
{
    /*to compute even pi  the six most significant bits in 10-bit sample*/
    /* Ri = P'i^Pi */
    return ((data&PARITY_BIT)>>10)^parity_check((data&(~PARITY_BIT))>>4);/*the 10bit is parity bit,and use the six most significant bits*/

}

INT32 parse_stereo_ra_rb(UINT8 *RA,UINT8* RB,UINT16 *data)
{
#if 0
    /* TO GET Ri*/
    *RA|=get_Ri(data[0])<<2;//R2A
    *RB|=get_Ri(data[1])<<2;//R2B
    *RA|=get_Ri(data[2])<<1;//R1A
    *RB|=get_Ri(data[3])<<1;//R1B
    *RA|=get_Ri(data[4])<<0;//R0A
    *RB|=get_Ri(data[5])<<0;//R0B
#else
    UINT8 i,max_data_a=0,max_data_b=0,data_ra=0,data_rb=0;
    UINT8 count_a[8]= {0,0,0,0,0,0,0,0},count_b[8]= {0,0,0,0,0,0,0,0};
    for(i=0; i<18; i++) {
        data_ra|=get_Ri(data[0])<<2;//R2A
        data_rb|=get_Ri(data[1])<<2;//R2B
        data_ra|=get_Ri(data[2])<<1;//R1A
        data_rb|=get_Ri(data[3])<<1;//R1B
        data_ra|=get_Ri(data[4])<<0;//R0A
        data_rb|=get_Ri(data[5])<<0;//R0B
        count_a[data_ra]++;
        count_b[data_rb]++;
        data_ra=0;
        data_rb=0;
    }
    for(i=0; i<8; i++) {
        if(max_data_a<count_a[i]) {
            max_data_a=count_a[i];
            *RA=i;
        }
        if(max_data_b<count_b[i]) {
            max_data_b=count_b[i];
            *RB=i;
        }
    }
#endif
    return 0;
}
UINT16 decode_data(UINT8 range,UINT16 data)
{

    UINT16 most_high_bit=(data & MOST_HIGH_BIT);
    data &=~(PARITY_BIT);// clear parity_bit
    switch(range&0x7) {
    case RANGE_1: {
        if(most_high_bit & MOST_HIGH_BIT) {
            data =((data<<4)&(0x1ff0))|(0xe000);
#if 0
            data =(data<<4)|(0xe000);
            if((data&0xfff)==0) {
                data &=~(0xfff);
            } else {
                data |=0xfff;
            }
#endif
        } else {
            data =((data<<4)&(0x1ff0));
            data &=~(0xe000);
#if 0
            data =((data<<4)&(0x1ff0));
            data &=~(0xe000);
            if((data&0xfff)==0) {
                data &=~(0xfff);
            } else {
                data |=0xfff;
            }
#endif
        }
    }
    break;
    case RANGE_2: {
        /*
        0 0 1xxxxxxxxx xxx
        1 1 0xxxxxxxxx xxx
        */
        if(most_high_bit & MOST_HIGH_BIT) {
            data =((data<<3)&(0x0ff8))|(0xf000);
        } else {
            data =((data<<3)&(0x0ff8));
            data &=~(0xf000);
        }

    }
    break;
    case RANGE_3: {
        /*
        0 00 1xxxxxxxxx xx
        1 11 0xxxxxxxxx xx
        */
        if(most_high_bit & MOST_HIGH_BIT) {
            data =((data<<2)&(0x07fc))|(0xf800);
        } else {
            data =((data<<2)&(0x07fc));
            data &=~(0xf800);
        }
    }
    break;
    case RANGE_4: {
        /*
        0 000 1xxxxxxxxx x
        1 111 0xxxxxxxxx x
        */
        if(most_high_bit & MOST_HIGH_BIT) {
            data =((data<<1)&(0x03fe))|(0xfc00);
        } else {
            data =((data<<1)&(0x03fe));
            data &=~(0xfc00);
        }
    }
    break;
    case RANGE_5_0:
    case RANGE_5_1:
    case RANGE_5_3:
    case RANGE_5_4: {
        /*
        0 0000 1xxxxxxxxx x
        1 1111 0xxxxxxxxx x
        */
        if(most_high_bit & MOST_HIGH_BIT) {
            data =(data&0x01ff)|(0xfe00);
        } else {
            data =(data&0x01ff);
            data &=~(0xfe00);
        }
    }
    break;
defalut:
    break;
    }
    return data;
}


INT32 parse_data_stereo_left_right(UINT8 RA,UINT8 RB,INT32 len,UINT16 *data,INT16 *left_data,INT16 *right_data,INT32 left_count,INT32 right_count)
{
    int i = 0;
    for(i = 0; i < len; i+=2) {
        left_data[left_count++]= decode_data(RA, data[i]);
        right_data[right_count++] = decode_data(RB, data[i+1]);
    }
    return left_count;
}

INT32 parse_mono_ra_rb(UINT8 *RA,UINT8* RB,UINT16 *data,UINT16 ctrl)
{
    UINT8 i,max_data=0,data_rab=0;
    UINT8 count[8]= {0,0,0,0,0,0,0,0};
    /* TO GET Ri*/
#if 0
    *RA|=get_Ri(data[0])<<2;//R2n
    *RA|=get_Ri(data[1])<<1;//R1n
    *RA|=get_Ri(data[2])<<0;//R0n
    *RB|=get_Ri(data[27])<<2;//R1B
    *RB|=get_Ri(data[28])<<1;//R0A
    *RB|=get_Ri(data[29])<<0;//R0B
#else
    for(i=0; i<18; i++) {
        data_rab|=get_Ri(data[0+i*3])<<2;
        data_rab|=get_Ri(data[1+i*3])<<1;
        data_rab|=get_Ri(data[2+i*3])<<0;
        count[data_rab]++;
        data_rab=0;
    }
    for(i=0; i<8; i++) {
        if(max_data<count[i]) {
            max_data=count[i];
            *RA=*RB=i;
        }
    }
#endif
    return 0;
}

INT32 parse_data_mono_data(UINT8 RA,UINT8 RB,INT32 len,UINT16 *data,INT16 *data_out,INT32 count)
{
    int i = 0;
    for(i = 0; i < len; i++) {
        data_out[count++]= decode_data(RA, data[i]);
    }

    return count;
}

INT32 parse_data_stereo(UINT8 RA,UINT8 RB,INT32 len,UINT16 *data)
{
    int i = 0;
    for(i = 0; i < len; i+=2) {
        data[i]= decode_data(RA, data[i]);
        data[i+1] = decode_data(RB, data[i+1]);
    }
    return 0;
}

UINT8 confirm_mon_dual_stereo(UINT8* data)
{
    UINT8 i,max_data=0,data_mds=0,index;
    UINT8 count[8]= {0,0,0,0,0,0,0,0};
    for(i=0; i<12; i++) {
        data_mds=(*(data+1+91*i)>>4)&(~(1<<3));
        count[data_mds]++;
        data_mds=0;
    }
    for(i=0; i<8; i++) {
        if(max_data<count[i]) {
            max_data=count[i];
            index=i;
        }
    }
    return index;
}

INT32 dev_app_nicam_decode_data(UINT8 *data,UINT32 size,INT16 *out_left,INT16 *out_right)
{
    INT32 i,j,k,n=0,m,ret=0;
    UINT8  RA,RB;
    UINT8 *p;
    UINT16 data_2[64];
    INT32 left_len=0,right_len=0;
    UINT32 *data_p=data;
    static INT32 mon_dual_stereo_tmp=0;
    if(err_nicam_data_num>=4) {
        ret=-1;
        mon_Event(0xb003ffff);
        goto OUT;
    }
    for(i=0; i<size; i++) {
        nicam_data[i]=(data_p[i]>>8)&0xff;
    }
    p=nicam_data_fix;
    ret =dev_app_read_frames(nicam_data,size,p);
    if(size==0)
	{
		 ret=0;
		goto OUT;
	}
    if(ret > 0) {
        i=ret;
    } else {
        mon_Event(0xb00300ff);
        err_nicam_data_num++;
        goto OUT;
    }
    j = i/91;
#if 1
    switch(mon_dual_stereo) {
    case NICAM_STEREO: {
        if(nicam_remain_num != 0) {
            left_len=right_len=copy_data_stereo_left_right(nicam_decoded_data,out_left,out_right,left_len,right_len,64);
        }
        break;
    }
    case NICAM_DUAL: {
        if(nicam_remain_num != 0) {
            if(flag_l_r&0x1) {
                right_len=copy_data_mono(nicam_decoded_data,out_right,64,right_len);
		//printf("lefit\n");
            } else {
                left_len=copy_data_mono(nicam_decoded_data,out_left,64,left_len);
				//printf("right\n");
            }
            flag_l_r++;
        }
        break;
    }
    case NICAM_MONO: {
        if(nicam_remain_num != 0) {
            if(flag_l_r&0x1) {
                //right_len=copy_data_mono(nicam_decoded_data,app_in_buff_right,64,right_len);
            } else {
                left_len=copy_data_mono(nicam_decoded_data,out_left,64,left_len);
                right_len=copy_data_mono(nicam_decoded_data,out_right,64,right_len);
            }
            flag_l_r++;
        }
        break;
    }
    default: {
        break;
    }
    }
#endif
    k=j;
#if 1
    if(((j+nicam_remain_num)&0x1)) {
        j=j-1;
    }
#endif
    if(flag_lr==0) {
        flag_l_r=confirm_left_right(p);
        flag_lr=1;
    }
    mon_dual_stereo=confirm_mon_dual_stereo(p);
    if(mon_dual_stereo_tmp!=mon_dual_stereo) {
        flag_lr=0;
        mon_Event(0xb003abff);
    }
    mon_dual_stereo_tmp = mon_dual_stereo;
    for(n=0; n<j; n++) {
        RA=RB=0;
        nicam728_descramble(p+1);
        swap_data16(p+3,44*2);
        nicam728_deinterleave((p+3),data_2);
        //mon_dual_stereo = (*(p+1)>>4)&(~(1<<3));
        //mon_dual_stereo=NICAM_MONO;

        switch(mon_dual_stereo) {
        case NICAM_STEREO: {
            parse_stereo_ra_rb(&RA,&RB,data_2);
            left_len=right_len=parse_data_stereo_left_right(RA,RB,64,data_2,out_left,out_right,left_len,right_len);
            break;
        }
        case NICAM_DUAL: {
            parse_mono_ra_rb(&RA,&RB,data_2,0);

            if(flag_l_r&0x1) {
                right_len=parse_data_mono_data(RA,RB,64,data_2,out_right,right_len);
		 //printf("left\n");
            } else {
                left_len=parse_data_mono_data(RA,RB,64,data_2,out_left,left_len);
		//printf("right \n");
            }
            flag_l_r++;
            break;
        }
        case NICAM_MONO: {
            parse_mono_ra_rb(&RA,&RB,data_2,0);
            if(flag_l_r&0x1) {
                //right_len=parse_data_mono_right(RA,RB,64,data_2,app_in_buff_right,right_len);
            } else {
                left_len=parse_data_mono_data(RA,RB,64,data_2,out_left,left_len);
                right_len=parse_data_mono_data(RA,RB,64,data_2,out_right,right_len);
            }

            flag_l_r++;
            break;
        }
        default: {
            mon_Event(0xb003abcf);
            break;
        }
        }

        p+=91;
    }
#if 1
    RA=RB=0;

    switch(mon_dual_stereo) {
    case NICAM_STEREO: {
        if(k!=j) {
            nicam728_descramble(p+1);
            swap_data16(p+3,44*2);
            nicam728_deinterleave((p+3),nicam_decoded_data);
            parse_stereo_ra_rb(&RA,&RB,nicam_decoded_data);
            parse_data_stereo(RA,RB,64,nicam_decoded_data);
            j=j+nicam_remain_num;
            nicam_remain_num=1;
        } else {
            j=j+nicam_remain_num;
            nicam_remain_num=0;
        }
        break;
    }
    case NICAM_DUAL: {
        if(k!=j) {
            nicam728_descramble(p+1);
            swap_data16(p+3,44*2);
            nicam728_deinterleave((p+3),nicam_decoded_data);
            parse_mono_ra_rb(&RA,&RB,nicam_decoded_data,0);
            parse_data_mono_data(RA,RB,64,nicam_decoded_data,(INT16 *)nicam_decoded_data,0);
            j=j+nicam_remain_num;
            nicam_remain_num=1;
        } else {
            j=j+nicam_remain_num;
            nicam_remain_num=0;
        }
        break;
    }
    case NICAM_MONO: {
        if(k!=j) {
            nicam728_descramble(p+1);
            swap_data16(p+3,44*2);
            nicam728_deinterleave((p+3),nicam_decoded_data);
            parse_mono_ra_rb(&RA,&RB,nicam_decoded_data,0);
            parse_data_mono_data(RA,RB,64,nicam_decoded_data,(INT16 *)nicam_decoded_data,0);
            j=j+nicam_remain_num;
            nicam_remain_num=1;
        } else {
            j=j+nicam_remain_num;
            nicam_remain_num=0;
        }
        break;
    }
    default: {
        mon_Event(0xb003abcf);
        break;
    }
    }
#endif
    ret=left_len*2;

OUT:
    return ret;
}

INT32 dev_app_nicam_get_mon_dual_stereo(VOID)
{
    return mon_dual_stereo;
}


int main(int argc, char* argv[])
{
	FILE *fp_in,*fp_out_l,*fp_out_r,*fp_test;
	int cnt,n,m,i;
	fp_in  = fopen("atv.bin","rb");//48_1K_16bit.bin 44_1_1k_16bit.bin 96_1k_16bit.bin resampe_test_8K.bin 16k_16bit.bin Aa22050-l-channel.pcm
	if (fp_in == NULL)
	{
		printf("aaaaab\n");
		return -1;
	}
	fp_out_l  = fopen("16bit_out_l.txt","w");//48_1K_16bit_out.bin 44_1K_16bit_out.bin 96_1K_16bit_out.bin 
	if (fp_out_l == NULL)
	{	
		printf("ddddd\n");
		return -2;
	}
	fp_out_r  = fopen("16bit_out_r.txt","w");//48_1K_16bit_out.bin 44_1K_16bit_out.bin 96_1K_16bit_out.bin 
	if (fp_out_r == NULL)
	{	
		printf("cccccc\n");
		return -2;
	}
	while(1)
	{
		n = fread(&av_ifc_buff_in,1,sizeof(av_ifc_buff_in)/2,fp_in);
		printf("read n = %d\n",n);
		if (n != sizeof(av_ifc_buff_in)/2)
		{
			m=dev_app_nicam_decode_data(av_ifc_buff_in,n/4,app_in_buff_left,app_in_buff_right);
			printf("m = %d\n",m);
			for(i=0;i<m/2;i++)
			{
				fprintf(fp_out_l,"%04d\n",app_in_buff_left[i]);
				printf(fp_out_r,"%04d\n",app_in_buff_right[i]);
			}
			printf("finished \n");
			break;
		}
		m=dev_app_nicam_decode_data(av_ifc_buff_in,n/4,app_in_buff_left,app_in_buff_right);
		for(i=0;i<m/2;i++)
		{
			fprintf(fp_out_l,"%04d\n",app_in_buff_left[i]);
			fprintf(fp_out_r,"%04d\n",app_in_buff_right[i]);
		}
	}
	
	fclose(fp_in);
	fclose(fp_out_l);
	fclose(fp_out_r);
	printf("Hello World!\n");
	return 0;
}

