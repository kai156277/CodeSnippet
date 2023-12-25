#include <QApplication>
#include <QDebug>
#include <QFileDialog>

//#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#include <spdlog/spdlog.h>

int main(int argc, char *argv[])
{
    spdlog::set_level(spdlog::level::debug);
    SPDLOG_DEBUG("debug: END");
    SPDLOG_TRACE("trace: END");
    SPDLOG_INFO("info: END");
    /*
    QApplication app(argc, argv);
    QStringList  pacp_files = QFileDialog::getOpenFileNames(nullptr, "get pacp files");

    char vlp_udp_page[1248];
    char pcap_header[40];
    for (int i = 0; i < pacp_files.size(); ++i)
    {
        qDebug() << "pacp file: " << pacp_files[i];
        QFile     pacp_raw(pacp_files[i]);
        QFileInfo pacp_file_info     = QFileInfo(pacp_files[i]);
        QString   no_suffix_filename = pacp_file_info.absolutePath() + "/" + pacp_file_info.baseName();
        QFile     pacp_2368(no_suffix_filename + "_2368.pcap");
        pacp_2368.open(QIODevice::WriteOnly);
        QFile pacp_2372(no_suffix_filename + "_2372.pcap");
        pacp_2372.open(QIODevice::WriteOnly);
        if (pacp_raw.open(QIODevice::ReadOnly))
        {
            pacp_raw.read(pcap_header, 40);
            pacp_2372.write(pcap_header);
            pacp_2368.write(pcap_header);
            while (!pacp_raw.atEnd())
            {
                pacp_raw.read(vlp_udp_page, 1248);
                if (vlp_udp_page[36] == (char) 0x09 && vlp_udp_page[37] == (char) 0x44)
                {
                    pacp_2372.write(vlp_udp_page);
                }
                if (vlp_udp_page[36] == (char) 0x09 && vlp_udp_page[37] == (char) 0x40)
                {
                    pacp_2368.write(vlp_udp_page);
                }
            }
        }
    }
    */

    return 0;
}
