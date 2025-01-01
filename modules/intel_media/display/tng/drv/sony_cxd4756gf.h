static struct reg_val {
	u16 reg;
	u8 val;
};
#ifdef SUPPORT_CADIZ
int __init cadiz_i2c_init(void);
void __exit cadiz_i2c_exit(void);
int cadiz_power(int);
int cadiz_reset(int);
int cadiz_enable_i2c(void);
int cadiz_wait_tx_init_done(void);
int cadiz_wait_trans_complete(void);
int cadiz_setup_boot1();
int cadiz_setup_boot2();
int cadiz_setup_boot3();
int cadiz_setup_boot4();
int cadiz_setup_ipc_ibc();
int cadiz_setup_regs(struct reg_val*, int);
void cadiz_pass_tables(struct reg_val*, int,
		       struct reg_val*, int,
		       struct reg_val*, int,
		       struct reg_val*, int,
		       struct reg_val*, int);
void cadiz_set_accessible(int);
int cadiz_need_reinit(void);
#else
static int cadiz_i2c_init(void)
{ return 0; }
static void cadiz_i2c_exit(void)
{ return; }
static int cadiz_power(int val)
{ return 0; }
static int cadiz_reset(int val)
{ return 0; }
static int cadiz_enable_i2c(void)
{ return 0; }
static int cadiz_wait_tx_init_done(void)
{ return 0; }
static int cadiz_wait_trans_complete(void)
{ return 0; }
static int cadiz_setup_boot1()
{ return 0; }
static int cadiz_setup_boot2()
{ return 0; }
static int cadiz_setup_boot3()
{ return 0; }
static int cadiz_setup_boot4()
{ return 0; }
static int cadiz_setup_ipc_ibc()
{ return 0; }
static int cadiz_setup_regs(struct reg_val* regs, int len)
{ return 0; }
static void cadiz_pass_tables(struct reg_val* b1,  int b1_len,
		       struct reg_val* b2,  int b2_len,
		       struct reg_val* b3,  int b3_len,
		       struct reg_val* b4,  int b4_len,
		       struct reg_val* ipb, int ipb_len)
{ return 0; }
static void cadiz_set_accessible(int yes)
{ return; }
static int cadiz_need_reinit(void)
{ return 0; }
#endif
