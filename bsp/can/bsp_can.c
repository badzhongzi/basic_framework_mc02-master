#include "bsp_can.h"
#include "main.h"
#include "memory.h"
#include "stdlib.h"
#include "bsp_dwt.h"
#include "bsp_log.h"

/* can instance ptrs storage, used for recv callback */
// 在CAN产生接收中断会遍历数组,选出hcan和rxid与发生中断的实例相同的那个,调用其回调函数
// @todo: 后续为每个CAN总线单独添加一个can_instance指针数组,提高回调查找的性能
static CANInstance *can_instance[CAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx; // 全局CAN实例索引,每次有新的模块注册会自增

/* ----------------two static function called by CANRegister()-------------------- */

/**
 * @brief 添加过滤器以实现对特定id的报文的接收,会被CANRegister()调用
 *        给CAN添加过滤器后,BxCAN会根据接收到的报文的id进行消息过滤,符合规则的id会被填入FIFO触发中断
 *        对于FDCAN，设置使用特定ID模式过滤。
 *
 * @note f407的bxCAN有28个过滤器,这里将其配置为前14个过滤器给CAN1使用,后14个被CAN2使用
 *       初始化时,奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
 *       注册到CAN1的模块使用过滤器0-13,CAN2使用过滤器14-27
 *       FDCAN的消息RAM是所有FDCAN外设共用的。
 *       H723系列FDCAN过滤器数量完全在CubeMX中自定义，因此先做一次检查，再添加即可。
 *
 * @attention 你不需要完全理解这个函数的作用,因为它主要是用于初始化,在开发过程中不需要关心底层的实现
 *            享受开发的乐趣吧!如果你真的想知道这个函数在干什么,请联系作者或自己查阅资料(请直接查阅官方的reference manual)
 *            FDCAN的教程较少，但是添加FDCAN的人已经发了一篇CSDN讲解了，可以参考一下
 *
 * @param _instance can instance owned by specific module
 */
static void CANAddFilter(CANInstance *_instance)
{

#ifdef FDCAN
	static uint8_t can1_filter_idx = 0, can2_filter_idx = 0 , can3_filter_idx = 0;
	//检查是否超出过滤器设定数量上限
	if(can1_filter_idx > hfdcan1.Init.StdFiltersNbr || can2_filter_idx>hfdcan2.Init.StdFiltersNbr || can3_filter_idx > hfdcan3.Init.StdFiltersNbr)
	{
		while(1)
		{
			//报错
		}
	}
	uint8_t *filter_idx_p;

	if(_instance->can_handle==&hfdcan1)
	{
		filter_idx_p=&can1_filter_idx;
	}
	else if(_instance->can_handle==&hfdcan2)
	{
		filter_idx_p=&can2_filter_idx;
	}
	else if(_instance->can_handle==&hfdcan3)
	{
		filter_idx_p=&can3_filter_idx;
	}
	else
	{
		while(1)
		{
			//报错
		}
	}

	FDCAN_FilterTypeDef fdcan_filter_conf;
	fdcan_filter_conf.FilterIndex=(*filter_idx_p)++;
	//使用单个ID模式
	fdcan_filter_conf.FilterType=FDCAN_FILTER_DUAL;
	fdcan_filter_conf.FilterConfig=(_instance->tx_id & 1) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;//奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
	fdcan_filter_conf.FilterID1=_instance->rx_id;
	fdcan_filter_conf.FilterID2=_instance->rx_id;
	fdcan_filter_conf.IdType=FDCAN_STANDARD_ID;
	fdcan_filter_conf.IsCalibrationMsg=0;
	//fdcan_filter_conf.RxBufferIndex=0;

	HAL_FDCAN_ConfigFilter(_instance->can_handle, &fdcan_filter_conf);

#else
	CAN_FilterTypeDef can_filter_conf;
	static uint8_t can1_filter_idx = 0, can2_filter_idx = 14; // 0-13给can1用,14-27给can2用

	can_filter_conf.FilterMode = CAN_FILTERMODE_IDLIST;                                                       // 使用id list模式,即只有将rxid添加到过滤器中才会接收到,其他报文会被过滤
	can_filter_conf.FilterScale = CAN_FILTERSCALE_16BIT;                                                      // 使用16位id模式,即只有低16位有效
	can_filter_conf.FilterFIFOAssignment = (_instance->tx_id & 1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;              // 奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
	can_filter_conf.SlaveStartFilterBank = 14;                                                                // 从第14个过滤器开始配置从机过滤器(在STM32的BxCAN控制器中CAN2是CAN1的从机)
	can_filter_conf.FilterIdLow = _instance->rx_id << 5;                                                      // 过滤器寄存器的低16位,因为使用STDID,所以只有低11位有效,高5位要填0
	can_filter_conf.FilterBank = _instance->can_handle == &hcan1 ? (can1_filter_idx++) : (can2_filter_idx++); // 根据can_handle判断是CAN1还是CAN2,然后自增
	can_filter_conf.FilterActivation = CAN_FILTER_ENABLE;                                                     // 启用过滤器

	HAL_CAN_ConfigFilter(_instance->can_handle, &can_filter_conf);
#endif

}

/**
 * @brief 在第一个CAN实例初始化的时候会自动调用此函数,启动CAN服务
 *
 * @note 此函数会启动CAN1并开启中断
 *       FDCAN的情况下，我们采用FIFO接收方式（而不是buffer），FIFO和buffer还有queue接收方式请自行查阅H723手册
 *       FDCAN比bxCAN多了一个全局过滤器，这里配置为全部拒绝，只接受指定ID。
 *       
 */
void CANServiceInit()
{
#ifdef FDCAN
	//可能不需要这么多中断
	uint32_t FDCAN_RXActiveITs = FDCAN_IT_RX_FIFO0_NEW_MESSAGE|FDCAN_IT_RX_FIFO0_FULL\
			|FDCAN_IT_RX_FIFO0_WATERMARK|FDCAN_IT_RX_FIFO0_MESSAGE_LOST \
			|FDCAN_IT_RX_FIFO1_NEW_MESSAGE| FDCAN_IT_RX_FIFO1_FULL\
			|FDCAN_IT_RX_FIFO1_WATERMARK|FDCAN_IT_RX_FIFO1_MESSAGE_LOST;


	//HAL_FDCAN_ConfigClockCalibration()
	HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1,FDCAN_RX_FIFO0,FDCAN_RX_FIFO_OVERWRITE);
	HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1,FDCAN_RX_FIFO1,FDCAN_RX_FIFO_OVERWRITE);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);//全局过滤器设置
	HAL_FDCAN_Start(&hfdcan1);
	HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_RXActiveITs, 0);

	HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan2,FDCAN_RX_FIFO0,FDCAN_RX_FIFO_OVERWRITE);
	HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan2,FDCAN_RX_FIFO1,FDCAN_RX_FIFO_OVERWRITE);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	HAL_FDCAN_Start(&hfdcan2);
	HAL_FDCAN_ActivateNotification(&hfdcan2,FDCAN_RXActiveITs, 0);


	HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan3,FDCAN_RX_FIFO0,FDCAN_RX_FIFO_OVERWRITE);
	HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan3,FDCAN_RX_FIFO1,FDCAN_RX_FIFO_OVERWRITE);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	HAL_FDCAN_Start(&hfdcan3);
	HAL_FDCAN_ActivateNotification(&hfdcan3,FDCAN_RXActiveITs, 0);


#else
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
#endif

}

/* ----------------------- two extern callable function -----------------------*/

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    if (!idx)
    {
        CANServiceInit(); // 第一次注册,先进行硬件初始化
        LOGINFO("[bsp_can] CAN Service Init");
    }
    if (idx >= CAN_MX_REGISTER_CNT) // 超过最大实例数
    {
        while (1)
        {
        	LOGERROR("[bsp_can] CAN instance exceeded MAX num, consider balance the load of CAN bus");
        }

    }
    for (size_t i = 0; i < idx; i++)
    { // 重复注册 | id重复
        if (can_instance[i]->rx_id == config->rx_id && can_instance[i]->can_handle == config->can_handle)
        {
            while (1)
            {
            	LOGERROR("[}bsp_can] CAN id crash ,tx [%d] or rx [%d] already registered", &config->tx_id, &config->rx_id);
            }

        }
    }

    CANInstance *instance = (CANInstance *)malloc(sizeof(CANInstance)); // 分配空间
    memset(instance, 0, sizeof(CANInstance));                           // 分配的空间未必是0,所以要先清空
    // 进行发送报文的配置
#ifdef FDCAN
    instance->txconf.Identifier = config->tx_id; 				// 发送id
    instance->txconf.IdType = FDCAN_STANDARD_ID;  				// 使用标准id,扩展id则使用CAN_ID_EXT(目前没有需求)
    instance->txconf.TxFrameType = FDCAN_DATA_FRAME,    		// 发送数据帧
    instance->txconf.DataLength = FDCAN_DLC_BYTES_8,    		// 数据长度为8字节
	instance->txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE,	// 兼容CAN2.0,错误状态指示器设为主动
	instance->txconf.BitRateSwitch = FDCAN_BRS_OFF,         	// 兼容CAN2.0禁用位速率切换
	instance->txconf.FDFormat = FDCAN_CLASSIC_CAN,          	// 使用经典CAN格式
	instance->txconf.TxEventFifoControl = FDCAN_NO_TX_EVENTS,	// 不需要，禁用事件FIFO
	instance->txconf.MessageMarker = 0;                     	// 不使用消息标记
#else
    instance->txconf.StdId = config->tx_id; // 发送id
    instance->txconf.IDE = CAN_ID_STD;      // 使用标准id,扩展id则使用CAN_ID_EXT(目前没有需求)
    instance->txconf.RTR = CAN_RTR_DATA;    // 发送数据帧
    instance->txconf.DLC = 0x08;            // 默认发送长度为8
#endif
    // 设置回调函数和接收发送id
    instance->can_handle = config->can_handle;
    instance->tx_id = config->tx_id; // 好像没用,可以删掉
    instance->rx_id = config->rx_id;
    instance->can_module_callback = config->can_module_callback;
    instance->id = config->id;

    CANAddFilter(instance);         // 添加CAN过滤器规则
    can_instance[idx++] = instance; // 将实例保存到can_instance中

    return instance; // 返回can实例指针
}

/* @todo 目前似乎封装过度,应该添加一个指向tx_buff的指针,tx_buff不应该由CAN instance保存 */
/* 如果让CANinstance保存txbuff,会增加一次复制的开销 */
uint8_t CANTransmit(CANInstance *_instance, float timeout)
{
    static uint32_t busy_count;
    static volatile float wait_time __attribute__((unused)); // for cancel warning
    float dwt_start = DWT_GetTimeline_ms();
#ifdef FDCAN
    while(HAL_FDCAN_GetTxFifoFreeLevel(_instance->can_handle)==0)
#else
    while (HAL_CAN_GetTxMailboxesFreeLevel(_instance->can_handle) == 0) // 等待邮箱空闲
#endif
    {
        if (DWT_GetTimeline_ms() - dwt_start > timeout) // 超时
        {
            LOGWARNING("[bsp_can] CAN MAILbox full! failed to add msg to mailbox. Cnt [%d]", busy_count);
            busy_count++;
            return 0;
        }
    }
    wait_time = DWT_GetTimeline_ms() - dwt_start;

#ifdef FDCAN
    if (HAL_FDCAN_AddMessageToTxFifoQ(_instance->can_handle, &_instance->txconf, _instance->tx_buff))
#else
    // tx_mailbox会保存实际填入了这一帧消息的邮箱,但是知道是哪个邮箱发的似乎也没啥用
    if (HAL_CAN_AddTxMessage(_instance->can_handle, &_instance->txconf, _instance->tx_buff, &_instance->tx_mailbox))
#endif
    {
        LOGWARNING("[bsp_can] CAN bus BUSY! cnt:%d", busy_count);
        busy_count++;
        return 0;
    }
    return 1; // 发送成功
}

void CANSetDLC(CANInstance *_instance, uint8_t length)
{
    // 发送长度错误!检查调用参数是否出错,或出现野指针/越界访问
    if (length > 8 || length == 0) // 安全检查
        while (1)
        {
        	LOGERROR("[bsp_can] CAN DLC error! check your code or wild pointer");
        }

    _instance->txconf.DataLength = DLC_LookUp_Table[length];
}

/* -----------------------belows are callback definitions--------------------------*/

//对于FDCAN，回调函数和处理方式完全不同，因此直接用两套逻辑处理
#ifdef FDCAN
/**
 * @brief 此函数会被下面两个函数调用,用于处理FIFO0和FIFO1溢出中断(说明收到了新的数据)
 *        所有的实例都会被遍历,找到can_handle和rx_id相等的实例时,调用该实例的回调函数
 *
 * @param _fdhcan
 * @param fifox passed to HAL_CAN_GetRxMessage() to get mesg from a specific fifo
 */
static void FDCANFIFOxCallback(FDCAN_HandleTypeDef *_hfdcan, uint32_t fifox)
{
    static FDCAN_RxHeaderTypeDef rxconf; // 同上
	static uint16_t DataLength = 0;
    static uint8_t fdcan_rx_buff[8];
    while (HAL_FDCAN_GetRxFifoFillLevel(_hfdcan, fifox)) // FIFO不为空,有可能在其他中断时有多帧数据进入
    {
        HAL_FDCAN_GetRxMessage(_hfdcan, fifox, &rxconf, fdcan_rx_buff); // 从FIFO中获取数据
		//解析数据长度，@Todo 此处在用新版本重新生成后可能得修改，DataLength可能不需要右移，具体情况具体看	！
		if(((rxconf.DataLength >> 16) & 0xF)>=0 && ((rxconf.DataLength >> 16) & 0xF)<=8)
		{
			DataLength=(rxconf.DataLength >> 16) & 0xF; // 保存接收到的数据长度
		}
		else
		{
			DataLength=0;
		}
        if(rxconf.RxFrameType==FDCAN_DATA_FRAME && rxconf.IdType==FDCAN_STANDARD_ID)
        {
        	for (size_t i = 0; i < idx; ++i)
			{
        		// 两者相等说明这是要找的实例
				if (_hfdcan == can_instance[i]->can_handle && rxconf.Identifier == can_instance[i]->rx_id)
				{
					if (can_instance[i]->can_module_callback != NULL) // 回调函数不为空就调用
					{
						can_instance[i]->rx_len = DataLength;               // 保存接收到的数据长度
						memcpy(can_instance[i]->rx_buff, fdcan_rx_buff, can_instance[i]->rx_len); // 消息拷贝到对应实例
						can_instance[i]->can_module_callback(can_instance[i]);     // 触发回调进行数据解析和处理
					}
					return;
				}
			}
        }
    }
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	/* 检查Rx FIFO 0中是否有消息丢失 */
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0)
	{
		//报错
	}
	/* 检查是否有新消息写入Rx FIFO 0或到达一定阈值 */
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)||(RxFifo0ITs & FDCAN_IT_RX_FIFO0_FULL)||(RxFifo0ITs & FDCAN_IT_RX_FIFO0_WATERMARK))
	{
		FDCANFIFOxCallback(hfdcan, FDCAN_RX_FIFO0); // 调用我们自己写的函数来处理消息
	}
}
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	/* 检查Rx FIFO 1中是否有消息丢失 */
	if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_MESSAGE_LOST) != 0)
	{
		//报错
	}
	/* 检查是否有新消息写入Rx FIFO 1或到达一定阈值 */
	if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE)||(RxFifo1ITs & FDCAN_IT_RX_FIFO1_FULL)||(RxFifo1ITs & FDCAN_IT_RX_FIFO1_WATERMARK))
	{
		FDCANFIFOxCallback(hfdcan, FDCAN_RX_FIFO1); // 调用我们自己写的函数来处理消息
	}
}


#else


/**
* @brief 此函数会被下面两个函数调用,用于处理FIFO0和FIFO1溢出中断(说明收到了新的数据)
*        所有的实例都会被遍历,找到can_handle和rx_id相等的实例时,调用该实例的回调函数
*
* @param _hcan
* @param fifox passed to HAL_CAN_GetRxMessage() to get mesg from a specific fifo
*/
static void CANFIFOxCallback(CAN_HandleTypeDef *_hcan, uint32_t fifox)
{
   static CAN_RxHeaderTypeDef rxconf; // 同上
   uint8_t can_rx_buff[8];
   while (HAL_CAN_GetRxFifoFillLevel(_hcan, fifox)) // FIFO不为空,有可能在其他中断时有多帧数据进入
   {
       HAL_CAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_buff); // 从FIFO中获取数据
       for (size_t i = 0; i < idx; ++i)
       { // 两者相等说明这是要找的实例
           if (_hcan == can_instance[i]->can_handle && rxconf.StdId == can_instance[i]->rx_id)
           {
               if (can_instance[i]->can_module_callback != NULL) // 回调函数不为空就调用
               {
                   can_instance[i]->rx_len = rxconf.DLC;                      // 保存接收到的数据长度
                   memcpy(can_instance[i]->rx_buff, can_rx_buff, rxconf.DLC); // 消息拷贝到对应实例
                   can_instance[i]->can_module_callback(can_instance[i]);     // 触发回调进行数据解析和处理
               }
               return;
           }
       }
   }
}

/**
* @brief 注意,STM32的两个CAN设备共享两个FIFO
* 下面两个函数是HAL库中的回调函数,他们被HAL声明为__weak,这里对他们进行重载(重写)
* 当FIFO0或FIFO1溢出时会调用这两个函数
*/
// 下面的函数会调用CANFIFOxCallback()来进一步处理来自特定CAN设备的消息

/**
* @brief rx fifo callback. Once FIFO_0 is full,this func would be called
*
* @param hcan CAN handle indicate which device the oddest mesg in FIFO_0 comes from
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
   CANFIFOxCallback(hcan, CAN_RX_FIFO0); // 调用我们自己写的函数来处理消息
}

/**
* @brief rx fifo callback. Once FIFO_1 is full,this func would be called
*
* @param hcan CAN handle indicate which device the oddest mesg in FIFO_1 comes from
*/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
   CANFIFOxCallback(hcan, CAN_RX_FIFO1); // 调用我们自己写的函数来处理消息
}


#endif
