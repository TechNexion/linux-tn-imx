/*===========================================================================
FILE:
   gobi_usbnet.h

DESCRIPTION:
   header for specific usbnet_tx_timeout and usbnet_start_xmit

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
===========================================================================*/

#if (LINUX_VERSION_CODE == KERNEL_VERSION( 2,6,31 ) ||\
       LINUX_VERSION_CODE == KERNEL_VERSION( 2,6,32 ))
void gobi_usbnet_tx_timeout_2_6_32 (struct net_device *net);
int gobi_usbnet_start_xmit_2_6_32 (struct sk_buff *skb, struct net_device *net);
#elif (LINUX_VERSION_CODE == KERNEL_VERSION( 2,6,35 ))
void gobi_usbnet_tx_timeout_2_6_35 (struct net_device *net);
netdev_tx_t gobi_usbnet_start_xmit_2_6_35 (struct sk_buff *skb,
                     struct net_device *net);
#elif (LINUX_VERSION_CODE == KERNEL_VERSION( 3,0,6 ))
void gobi_usbnet_tx_timeout_3_0_6 (struct net_device *net);
netdev_tx_t gobi_usbnet_start_xmit_3_0_6 (struct sk_buff *skb,
                     struct net_device *net);
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,10,1 ) &&\
       LINUX_VERSION_CODE <= KERNEL_VERSION( 3,10,39 ))
void gobi_usbnet_tx_timeout_3_10_21 (struct net_device *net);
netdev_tx_t gobi_usbnet_start_xmit_3_10_21 (struct sk_buff *skb,
                     struct net_device *net);
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,12,0 ) &&\
       LINUX_VERSION_CODE < KERNEL_VERSION( 3,13,0 ))
void gobi_usbnet_tx_timeout_3_12_xx(struct net_device *net);
netdev_tx_t gobi_usbnet_start_xmit_3_12_xx (struct sk_buff *skb,
                     struct net_device *net);
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION( 4,4,0 ) &&\
          LINUX_VERSION_CODE < KERNEL_VERSION( 4,5,0 ))
void gobi_usbnet_tx_timeout_4_4_xx(struct net_device *net);
netdev_tx_t gobi_usbnet_start_xmit_4_4_xx(struct sk_buff *skb,
                     struct net_device *net);

#else
#endif
