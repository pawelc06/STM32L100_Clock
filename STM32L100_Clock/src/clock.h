/* Private variables ---------------------------------------------------------*/
RTC_InitTypeDef  RTC_InitStructure;
RTC_TimeTypeDef  RTC_TimeStructure;



static void RTC_Config(void);
static void RTC_AlarmConfig(void);
static void RTC_Config32768Internal(void);
uint8_t bcd2dec(uint8_t numberbcd);
uint8_t dec2bcd(uint8_t numberdec);

void displayDate();
void updateAndDisplayDate();
void displayTime();

